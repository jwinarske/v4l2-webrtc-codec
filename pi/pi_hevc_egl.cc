// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Tier-0 confirmation: decode an HEVC frame on the Pi's stateless decoder
// (rpi-hevc-dec), import the resulting SAND128-tiled NV12 dma-buf into EGL as
// an EGL_LINUX_DMA_BUF_EXT image with its fourcc + modifier, sample it through
// a GL_OES_EGL_image_external texture into an offscreen FBO, and read the RGBA
// back. The read-back is checked against a CPU reference: the same buffer is
// detiled and run through the YUV->RGB matrix by hand.
//
// To keep the verdict honest, the tool also samples a control image: the very
// same pixels re-packed into a plain LINEAR NV12 dma-buf. The control isolates
// the one variable that matters -- the SAND128 tiling. If the control matches
// the CPU reference but the SAND import does not, the harness is sound and the
// GPU simply mis-detiles SAND128; if neither matches, the harness is suspect.
//
// Headless: renders on the render node (/dev/dri/renderD128) via a surfaceless
// GBM/EGL context, so it needs no DRM master and never touches the display.

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <fcntl.h>
#include <gbm.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "parse/h265/nal.h"
#include "src/v4l2_stateless_h265_decoder.h"

using namespace v4l2wc;

namespace {

#define LOGE(...) std::fprintf(stderr, __VA_ARGS__)

constexpr std::uint32_t kNv12 = 0x3231564E;  // DRM_FORMAT_NV12

std::vector<std::uint8_t> ReadFile(const char* path) {
  std::vector<std::uint8_t> out;
  FILE* f = std::fopen(path, "rb");
  if (!f) return out;
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (n > 0) {
    out.resize(static_cast<size_t>(n));
    if (std::fread(out.data(), 1, out.size(), f) != out.size()) out.clear();
  }
  std::fclose(f);
  return out;
}

// Access units split at the first non-VCL after a VCL, or a first-slice VCL.
std::vector<std::vector<std::uint8_t>> SplitAccessUnits(
    const std::vector<std::uint8_t>& data) {
  auto nals = h265::ParseAnnexB(data.data(), data.size());
  std::vector<std::vector<std::uint8_t>> aus;
  std::vector<std::uint8_t> cur;
  bool have_vcl = false;
  for (const auto& n : nals) {
    const bool vcl = h265::IsVcl(n.type);
    const bool first = vcl && !n.rbsp.empty() && (n.rbsp[0] & 0x80);
    if (have_vcl && (!vcl || first)) {
      aus.push_back(cur);
      cur.clear();
      have_vcl = false;
    }
    cur.insert(cur.end(), {0, 0, 1});
    cur.insert(cur.end(), n.raw.begin(), n.raw.end());
    if (vcl) have_vcl = true;
  }
  if (!cur.empty()) aus.push_back(cur);
  return aus;
}

// Detiles one NV12_COL128 plane (128-byte columns) into raster; `w_bytes` is
// the raster width in bytes, `col_h` the tile column height in rows.
void Detile(const std::uint8_t* t, std::uint32_t w_bytes, std::uint32_t h,
            std::uint32_t col_h, std::vector<std::uint8_t>* o) {
  o->resize(static_cast<size_t>(w_bytes) * h);
  for (std::uint32_t y = 0; y < h; ++y)
    for (std::uint32_t bx = 0; bx < w_bytes; ++bx)
      (*o)[static_cast<size_t>(y) * w_bytes + bx] =
          t[(bx / 128) * 128 * col_h + y * 128 + (bx % 128)];
}

// Limited-range 8-bit YUV -> RGB for the two matrices a driver is likely to
// pick for NV12; the read-back is matched against whichever fits best.
void YuvToRgb(bool bt709, int Y, int U, int V, int* r, int* g, int* b) {
  const double c = 1.16438 * (Y - 16);
  const double d = U - 128, e = V - 128;
  double rr, gg, bb;
  if (bt709) {
    rr = c + 1.79274 * e;
    gg = c - 0.21325 * d - 0.53291 * e;
    bb = c + 2.11240 * d;
  } else {  // BT.601
    rr = c + 1.59603 * e;
    gg = c - 0.39176 * d - 0.81297 * e;
    bb = c + 2.01723 * d;
  }
  *r = std::clamp(static_cast<int>(std::lround(rr)), 0, 255);
  *g = std::clamp(static_cast<int>(std::lround(gg)), 0, 255);
  *b = std::clamp(static_cast<int>(std::lround(bb)), 0, 255);
}

GLuint Compile(GLenum stage, const char* src) {
  GLuint s = glCreateShader(stage);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024];
    glGetShaderInfoLog(s, sizeof(log), nullptr, log);
    LOGE("shader compile failed: %s\n", log);
    return 0;
  }
  return s;
}

const char* kVert =
    "attribute vec2 pos;\n"
    "varying vec2 uv;\n"
    "void main(){ uv = pos * 0.5 + 0.5; gl_Position = vec4(pos, 0.0, 1.0); }\n";

const char* kFrag =
    "#extension GL_OES_EGL_image_external : require\n"
    "precision mediump float;\n"
    "uniform samplerExternalOES tex;\n"
    "varying vec2 uv;\n"
    "void main(){ gl_FragColor = texture2D(tex, uv); }\n";

struct Ext {
  PFNEGLCREATEIMAGEKHRPROC create = nullptr;
  PFNEGLDESTROYIMAGEKHRPROC destroy = nullptr;
  PFNGLEGLIMAGETARGETTEXTURE2DOESPROC target = nullptr;
};

struct Plane {
  int fd = -1;
  std::uint32_t offset = 0;
  std::uint32_t pitch = 0;
};

// Import an NV12 dma-buf (1 shared fd or 2 fds) as an external texture, sample
// it into a WxH RGBA FBO, and return the read-back. Empty on failure.
std::vector<std::uint8_t> SampleDmabuf(EGLDisplay dpy, const Ext& ext,
                                       GLuint prog, std::uint64_t modifier,
                                       const Plane& y, const Plane& uv,
                                       std::uint32_t w, std::uint32_t h) {
  const std::uint32_t lo = modifier & 0xffffffffU;
  const std::uint32_t hi = (modifier >> 32) & 0xffffffffU;
  const EGLint attrs[] = {EGL_WIDTH,
                          static_cast<EGLint>(w),
                          EGL_HEIGHT,
                          static_cast<EGLint>(h),
                          EGL_LINUX_DRM_FOURCC_EXT,
                          static_cast<EGLint>(kNv12),
                          EGL_DMA_BUF_PLANE0_FD_EXT,
                          y.fd,
                          EGL_DMA_BUF_PLANE0_OFFSET_EXT,
                          static_cast<EGLint>(y.offset),
                          EGL_DMA_BUF_PLANE0_PITCH_EXT,
                          static_cast<EGLint>(y.pitch),
                          EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT,
                          static_cast<EGLint>(lo),
                          EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT,
                          static_cast<EGLint>(hi),
                          EGL_DMA_BUF_PLANE1_FD_EXT,
                          uv.fd,
                          EGL_DMA_BUF_PLANE1_OFFSET_EXT,
                          static_cast<EGLint>(uv.offset),
                          EGL_DMA_BUF_PLANE1_PITCH_EXT,
                          static_cast<EGLint>(uv.pitch),
                          EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT,
                          static_cast<EGLint>(lo),
                          EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT,
                          static_cast<EGLint>(hi),
                          EGL_YUV_COLOR_SPACE_HINT_EXT,
                          EGL_ITU_REC709_EXT,
                          EGL_SAMPLE_RANGE_HINT_EXT,
                          EGL_YUV_NARROW_RANGE_EXT,
                          EGL_NONE};
  EGLImageKHR image =
      ext.create(dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attrs);
  if (image == EGL_NO_IMAGE_KHR) {
    LOGE("eglCreateImageKHR failed: 0x%x\n", eglGetError());
    return {};
  }

  GLuint tex = 0;
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, tex);
  ext.target(GL_TEXTURE_EXTERNAL_OES, static_cast<GLeglImageOES>(image));
  // Point sampling so the read-back matches a nearest-neighbour CPU reference.
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  GLuint rgba = 0;
  glGenTextures(1, &rgba);
  glBindTexture(GL_TEXTURE_2D, rgba);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  GLuint fbo = 0;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         rgba, 0);
  std::vector<std::uint8_t> px;
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(prog);
    glUniform1i(glGetUniformLocation(prog, "tex"), 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, tex);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    px.resize(static_cast<size_t>(w) * h * 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    if (glGetError() != GL_NO_ERROR) px.clear();
  } else {
    LOGE("FBO incomplete\n");
  }

  glDeleteFramebuffers(1, &fbo);
  glDeleteTextures(1, &rgba);
  glDeleteTextures(1, &tex);
  ext.destroy(dpy, image);
  return px;
}

struct FitResult {
  const char* name = "";
  double mean = 1e9;
  int worst = 255;
  double lstd = 0;
};

// Match a read-back against the CPU YUV reference over an interior grid, trying
// both matrices and both vertical orientations; return the closest fit and the
// read-back's luma spread (to tell real content from a flat fill).
FitResult EvaluateFit(const std::vector<std::uint8_t>& px,
                      const std::vector<std::uint8_t>& yr,
                      const std::vector<std::uint8_t>& uvr, std::uint32_t W,
                      std::uint32_t H, const char* tag) {
  struct Cand {
    const char* name;
    bool bt709;
    bool flip;
    double mean;
    int worst;
  };
  Cand cs[4] = {{"BT.709 top-down", true, false, 0, 0},
                {"BT.709 flipped", true, true, 0, 0},
                {"BT.601 top-down", false, false, 0, 0},
                {"BT.601 flipped", false, true, 0, 0}};
  const std::uint32_t x0 = W / 10, x1 = W - W / 10;
  const std::uint32_t y0 = H / 10, y1 = H - H / 10;
  for (auto& c : cs) {
    double acc = 0;
    int worst = 0;
    std::uint64_t cnt = 0;
    for (std::uint32_t y = y0; y < y1; y += 3)
      for (std::uint32_t x = x0; x < x1; x += 3) {
        const std::uint32_t sy = c.flip ? (H - 1 - y) : y;
        const int Y = yr[static_cast<size_t>(sy) * W + x];
        const int U = uvr[static_cast<size_t>(sy / 2) * W + (x / 2) * 2];
        const int V = uvr[static_cast<size_t>(sy / 2) * W + (x / 2) * 2 + 1];
        int r, g, b;
        YuvToRgb(c.bt709, Y, U, V, &r, &g, &b);
        const size_t i = (static_cast<size_t>(y) * W + x) * 4;
        const int dr = std::abs(px[i] - r), dg = std::abs(px[i + 1] - g),
                  db = std::abs(px[i + 2] - b);
        acc += dr + dg + db;
        worst = std::max({worst, dr, dg, db});
        ++cnt;
      }
    c.mean = cnt ? acc / (cnt * 3.0) : 1e9;
    c.worst = worst;
  }
  double lsum = 0, lsq = 0;
  std::uint64_t ln = 0;
  for (std::uint32_t y = y0; y < y1; y += 7)
    for (std::uint32_t x = x0; x < x1; x += 7) {
      const size_t i = (static_cast<size_t>(y) * W + x) * 4;
      const double l = 0.299 * px[i] + 0.587 * px[i + 1] + 0.114 * px[i + 2];
      lsum += l;
      lsq += l * l;
      ++ln;
    }
  const double lmean = lsum / ln;
  const double lstd = std::sqrt(std::max(0.0, lsq / ln - lmean * lmean));

  const Cand* best = &cs[0];
  for (const auto& c : cs)
    if (c.mean < best->mean) best = &c;
  LOGE("[%s] luma mean=%.1f std=%.1f\n", tag, lmean, lstd);
  for (const auto& c : cs)
    LOGE("    %-16s mean|d|=%.2f  worst=%d\n", c.name, c.mean, c.worst);
  return {best->name, best->mean, best->worst, lstd};
}

// Dump an RGBA read-back as a binary PPM for eyeballing.
void DumpPpm(const char* path, const std::vector<std::uint8_t>& px,
             std::uint32_t W, std::uint32_t H) {
  FILE* f = std::fopen(path, "wb");
  if (!f) return;
  std::fprintf(f, "P6\n%u %u\n255\n", W, H);
  for (std::uint32_t y = 0; y < H; ++y)
    for (std::uint32_t x = 0; x < W; ++x)
      std::fwrite(&px[(static_cast<size_t>(y) * W + x) * 4], 1, 3, f);
  std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    LOGE("usage: %s <clip.hevc> [video_node] [render_node] [ppm_prefix]\n",
         argv[0]);
    return 2;
  }
  const char* node = argc > 2 ? argv[2] : "/dev/video19";
  const char* render = argc > 3 ? argv[3] : "/dev/dri/renderD128";

  // --- decode one frame; keep it (no Release) so its dma-buf stays valid ---
  std::vector<std::uint8_t> data = ReadFile(argv[1]);
  if (data.empty()) {
    LOGE("cannot read %s\n", argv[1]);
    return 1;
  }
  auto decoder = V4l2StatelessH265Decoder::Create(node, 16);
  if (!decoder) {
    LOGE("decoder create failed on %s\n", node);
    return 1;
  }
  V4l2DmaFrame frame;
  bool got = false;
  std::uint64_t ts = 0;
  for (auto& au : SplitAccessUnits(data)) {
    decoder->SubmitBitstream(au.data(), au.size(), ts++);
    if (decoder->Acquire(&frame)) {
      got = true;
      break;
    }
  }
  if (!got) {
    LOGE("no frame decoded\n");
    return 1;
  }
  const std::uint32_t W = frame.width, H = frame.height;
  LOGE("decoded frame: %ux%u fourcc=%c%c%c%c modifier=0x%llx planes=%u\n", W, H,
       frame.drm_fourcc & 0xff, (frame.drm_fourcc >> 8) & 0xff,
       (frame.drm_fourcc >> 16) & 0xff, (frame.drm_fourcc >> 24) & 0xff,
       static_cast<unsigned long long>(frame.modifier), frame.num_planes);
  if (frame.drm_fourcc != kNv12) {
    LOGE("this Tier-0 sample handles 8-bit NV12; got a non-NV12 frame\n");
    return 1;
  }

  // --- headless EGL on the render node via GBM ---
  int rfd = ::open(render, O_RDWR | O_CLOEXEC);
  if (rfd < 0) {
    LOGE("open %s failed\n", render);
    return 1;
  }
  gbm_device* gbm = gbm_create_device(rfd);
  if (!gbm) {
    LOGE("gbm_create_device failed\n");
    return 1;
  }
  auto get_plat = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
      eglGetProcAddress("eglGetPlatformDisplayEXT"));
  EGLDisplay dpy = EGL_NO_DISPLAY;
  if (get_plat) dpy = get_plat(EGL_PLATFORM_GBM_KHR, gbm, nullptr);
  if (dpy == EGL_NO_DISPLAY)
    dpy = eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(gbm));
  if (dpy == EGL_NO_DISPLAY) {
    LOGE("eglGetDisplay failed\n");
    return 1;
  }
  EGLint major = 0, minor = 0;
  if (!eglInitialize(dpy, &major, &minor)) {
    LOGE("eglInitialize failed\n");
    return 1;
  }
  const char* exts = eglQueryString(dpy, EGL_EXTENSIONS);
  const bool has_import =
      exts && std::strstr(exts, "EGL_EXT_image_dma_buf_import");
  LOGE("EGL %d.%d, EGL_EXT_image_dma_buf_import=%s\n", major, minor,
       has_import ? "yes" : "NO");
  if (!has_import) {
    LOGE("driver cannot import dma-buf; Tier-0 not available here\n");
    return 1;
  }

  eglBindAPI(EGL_OPENGL_ES_API);
  // GBM/Mesa exposes window-type configs; we render only to an FBO under a
  // surfaceless context, so the config's surface type is immaterial -- just
  // pick a matching one to hang the ES2 context on.
  const EGLint cfg_attrs[] = {EGL_SURFACE_TYPE,
                              EGL_WINDOW_BIT,
                              EGL_RENDERABLE_TYPE,
                              EGL_OPENGL_ES2_BIT,
                              EGL_RED_SIZE,
                              8,
                              EGL_GREEN_SIZE,
                              8,
                              EGL_BLUE_SIZE,
                              8,
                              EGL_NONE};
  EGLConfig cfg = nullptr;
  EGLint ncfg = 0;
  if (!eglChooseConfig(dpy, cfg_attrs, &cfg, 1, &ncfg) || ncfg == 0) {
    LOGE("eglChooseConfig failed\n");
    return 1;
  }
  const EGLint ctx_attrs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
  EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attrs);
  if (ctx == EGL_NO_CONTEXT) {
    LOGE("eglCreateContext failed\n");
    return 1;
  }
  if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
    LOGE("eglMakeCurrent (surfaceless) failed: 0x%x\n", eglGetError());
    return 1;
  }
  LOGE("GL_RENDERER: %s\n", glGetString(GL_RENDERER));
  const char* gl_exts =
      reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS));
  if (!gl_exts || !std::strstr(gl_exts, "GL_OES_EGL_image_external")) {
    LOGE("no GL_OES_EGL_image_external; cannot sample the imported image\n");
    return 1;
  }

  Ext ext;
  ext.create = reinterpret_cast<PFNEGLCREATEIMAGEKHRPROC>(
      eglGetProcAddress("eglCreateImageKHR"));
  ext.destroy = reinterpret_cast<PFNEGLDESTROYIMAGEKHRPROC>(
      eglGetProcAddress("eglDestroyImageKHR"));
  ext.target = reinterpret_cast<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>(
      eglGetProcAddress("glEGLImageTargetTexture2DOES"));
  if (!ext.create || !ext.destroy || !ext.target) {
    LOGE("missing EGLImage entry points\n");
    return 1;
  }

  // --- one ES2 program + a fullscreen triangle, reused for both samples ---
  GLuint vs = Compile(GL_VERTEX_SHADER, kVert);
  GLuint fs = Compile(GL_FRAGMENT_SHADER, kFrag);
  if (!vs || !fs) return 1;
  GLuint prog = glCreateProgram();
  glAttachShader(prog, vs);
  glAttachShader(prog, fs);
  glBindAttribLocation(prog, 0, "pos");
  glLinkProgram(prog);
  GLint linked = 0;
  glGetProgramiv(prog, GL_LINK_STATUS, &linked);
  if (!linked) {
    char log[1024];
    glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
    LOGE("link failed: %s\n", log);
    return 1;
  }
  const GLfloat tri[] = {-1.f, -1.f, 3.f, -1.f, -1.f, 3.f};
  GLuint vbo = 0;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(tri), tri, GL_STATIC_DRAW);

  // --- CPU reference: detile the decoded buffer (the known-good ground truth)
  // ---
  struct stat st0{};
  struct stat st1{};
  fstat(frame.fds[0], &st0);
  fstat(frame.fds[1], &st1);
  auto* y_map = static_cast<std::uint8_t*>(
      mmap(nullptr, st0.st_size, PROT_READ, MAP_SHARED, frame.fds[0], 0));
  auto* uv_map = static_cast<std::uint8_t*>(
      mmap(nullptr, st1.st_size, PROT_READ, MAP_SHARED, frame.fds[1], 0));
  if (y_map == MAP_FAILED || uv_map == MAP_FAILED) {
    LOGE("mmap of decoded planes failed\n");
    return 1;
  }
  const std::uint32_t col_h =
      static_cast<std::uint32_t>((frame.modifier >> 8) & 0xffffffffffffULL);
  const std::uint32_t lch = col_h ? col_h : H;
  std::vector<std::uint8_t> yr, uvr;
  Detile(y_map, W, H, lch, &yr);
  Detile(uv_map, W, H / 2, lch / 2, &uvr);

  // --- sample #1: the decoder's SAND128 dma-buf, straight in ---
  const std::vector<std::uint8_t> px_sand =
      SampleDmabuf(dpy, ext, prog, frame.modifier,
                   {frame.fds[0], frame.offsets[0], frame.pitches[0]},
                   {frame.fds[1], frame.offsets[1], frame.pitches[1]}, W, H);
  if (px_sand.empty()) {
    LOGE("SAND128 import/sample failed outright\n");
    return 1;
  }
  LOGE("imported + sampled the decoder's SAND128 dma-buf\n");
  const FitResult sand = EvaluateFit(px_sand, yr, uvr, W, H, "SAND128");

  // --- control: repack the same pixels into a LINEAR NV12 dma-buf ---
  // v3d/gbm won't allocate a planar NV12 bo, so build the two NV12 planes as
  // separate LINEAR R8 buffers (Y is W x H, the interleaved UV is W x H/2) and
  // import them together as one NV12 image -- a standard two-fd NV12 import.
  FitResult ctrl;
  bool ctrl_ran = false;
  auto make_r8 = [&](std::uint32_t w, std::uint32_t h) -> gbm_bo* {
    if (gbm_bo* b = gbm_bo_create(gbm, w, h, GBM_FORMAT_R8,
                                  GBM_BO_USE_LINEAR | GBM_BO_USE_RENDERING))
      return b;
    return gbm_bo_create(gbm, w, h, GBM_FORMAT_R8, GBM_BO_USE_LINEAR);
  };
  gbm_bo* ybo = make_r8(W, H);
  gbm_bo* uvbo = make_r8(W, H / 2);
  if (!ybo || !uvbo) {
    LOGE("control skipped: gbm_bo_create(R8, LINEAR) unsupported here\n");
  } else {
    auto fill = [](gbm_bo* bo, const std::uint8_t* src, std::uint32_t w,
                   std::uint32_t h) -> bool {
      std::uint32_t stride = 0;
      void* map_data = nullptr;
      void* addr =
          gbm_bo_map(bo, 0, 0, w, h, GBM_BO_TRANSFER_WRITE, &stride, &map_data);
      if (!addr || addr == MAP_FAILED) return false;
      auto* m = static_cast<std::uint8_t*>(addr);
      for (std::uint32_t y = 0; y < h; ++y)
        std::memcpy(m + static_cast<size_t>(y) * stride,
                    src + static_cast<size_t>(y) * w, w);
      gbm_bo_unmap(bo, map_data);
      return true;
    };
    if (fill(ybo, yr.data(), W, H) && fill(uvbo, uvr.data(), W, H / 2)) {
      int yfd = gbm_bo_get_fd(ybo);
      int uvfd = gbm_bo_get_fd(uvbo);
      const std::vector<std::uint8_t> px_lin =
          SampleDmabuf(dpy, ext, prog, gbm_bo_get_modifier(ybo),
                       {yfd, 0, gbm_bo_get_stride(ybo)},
                       {uvfd, 0, gbm_bo_get_stride(uvbo)}, W, H);
      ::close(yfd);
      ::close(uvfd);
      if (!px_lin.empty()) {
        LOGE("imported + sampled a LINEAR NV12 rebuild (modifier=0x%llx)\n",
             static_cast<unsigned long long>(gbm_bo_get_modifier(ybo)));
        ctrl = EvaluateFit(px_lin, yr, uvr, W, H, "LINEAR");
        ctrl_ran = true;
        if (argc > 4) {
          char path[512];
          std::snprintf(path, sizeof(path), "%s_lin.ppm", argv[4]);
          DumpPpm(path, px_lin, W, H);
        }
      }
    } else {
      LOGE("control skipped: mmap of gbm bo failed\n");
    }
  }
  if (ybo) gbm_bo_destroy(ybo);
  if (uvbo) gbm_bo_destroy(uvbo);

  if (argc > 4) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s_sand.ppm", argv[4]);
    DumpPpm(path, px_sand, W, H);
    // The CPU reference, for side-by-side comparison.
    std::vector<std::uint8_t> ref(static_cast<size_t>(W) * H * 4, 255);
    for (std::uint32_t y = 0; y < H; ++y)
      for (std::uint32_t x = 0; x < W; ++x) {
        int r, g, b;
        YuvToRgb(true, yr[static_cast<size_t>(y) * W + x],
                 uvr[static_cast<size_t>(y / 2) * W + (x / 2) * 2],
                 uvr[static_cast<size_t>(y / 2) * W + (x / 2) * 2 + 1], &r, &g,
                 &b);
        const size_t i = (static_cast<size_t>(y) * W + x) * 4;
        ref[i] = r;
        ref[i + 1] = g;
        ref[i + 2] = b;
      }
    std::snprintf(path, sizeof(path), "%s_cpu.ppm", argv[4]);
    DumpPpm(path, ref, W, H);
    LOGE("wrote %s_{sand,lin,cpu}.ppm\n", argv[4]);
  }

  // A read-back matches the CPU convert to within a few codes (rounding, chroma
  // siting). worst<48 guards against a scramble hiding behind a low mean.
  auto ok = [](const FitResult& f) {
    return f.mean < 6.0 && f.worst < 48 && f.lstd > 3.0;
  };
  const bool sand_ok = ok(sand);
  const bool ctrl_ok = ctrl_ran && ok(ctrl);

  LOGE("----\n");
  if (ctrl_ran)
    LOGE("control (LINEAR NV12): %s  best=%s mean|d|=%.2f worst=%d\n",
         ctrl_ok ? "PASS" : "FAIL", ctrl.name, ctrl.mean, ctrl.worst);
  else
    LOGE("control (LINEAR NV12): not run\n");
  LOGE("subject (SAND128)     : %s  best=%s mean|d|=%.2f worst=%d\n",
       sand_ok ? "PASS" : "FAIL", sand.name, sand.mean, sand.worst);

  int rc;
  if (sand_ok) {
    LOGE(
        "=== TIER-0 PASS: zero-copy GL sampling of the decoder's SAND128 "
        "output is pixel-correct ===\n");
    rc = 0;
  } else if (ctrl_ok) {
    LOGE(
        "=== TIER-0 NEGATIVE: the harness is sound (LINEAR control matches) "
        "but this GPU mis-samples SAND128 -- zero-copy GL texturing of the "
        "decoder output is not correct here; use the KMS-plane path (Tier 1) "
        "or detile first ===\n");
    rc = 2;
  } else {
    LOGE(
        "=== TIER-0 INCONCLUSIVE: neither sample matched; harness or "
        "environment problem ===\n");
    rc = 1;
  }

  glDeleteProgram(prog);
  glDeleteBuffers(1, &vbo);
  eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  eglDestroyContext(dpy, ctx);
  eglTerminate(dpy);
  gbm_device_destroy(gbm);
  ::close(rfd);
  return rc;
}

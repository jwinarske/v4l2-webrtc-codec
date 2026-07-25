// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Producer-side seam for the host->encode path: take a GPU-rendered RGBA frame
// (the stand-in for a Flutter offscreen surface), convert it to NV12 on the GPU
// straight into a single dma-buf, and feed that buffer zero-copy to the V4L2
// hardware H.264 encoder (/dev/video11 on the Pi). It is the inverse of the
// Tier-0 sampler: there we imported the decoder's NV12 and sampled it to RGBA;
// here we render RGBA and pack it into NV12 for the encoder.
//
// The NV12 buffer is one contiguous dma-heap allocation (Y at 0, interleaved
// CbCr at stride*height) because that is exactly what the encoder's OUTPUT
// queue imports -- a single fd of stride*height*3/2 bytes. It is imported as a
// single tall R8 EGL image and packed by one fragment shader (luma in the top
// H rows, chroma in the bottom H/2), so there is no two-plane / channel-order
// ambiguity to get wrong.
//
// Correctness is checked two ways, independent of the encoder: the packed NV12
// is read back on the CPU and compared against a CPU RGB->NV12 of the same
// source, and the coded output is checked for a keyframe and H.264 start codes.
// If rendering into the imported dma-buf is unsupported, the tool falls back to
// glReadPixels into the mapped buffer and says so, so the encoder path is still
// exercised and the limitation is explicit.
//
// Headless: renders on the render node via a surfaceless GBM/EGL ES3 context.

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>
#include <fcntl.h>
#include <gbm.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "src/v4l2_m2m_encoder.h"

using v4l2wc::V4l2M2mEncoder;

namespace {

#define LOGE(...) std::fprintf(stderr, __VA_ARGS__)

constexpr std::uint32_t kR8 = 0x20203852;  // DRM_FORMAT_R8 ('R','8',' ',' ')

// ---- source pattern (stands in for a Flutter-rendered RGBA surface) --------

// SMPTE-ish vertical color bars with a box that slides each frame, so the
// encoder sees real motion (delta frames) rather than a static fill.
void FillRgba(std::uint8_t* p, std::uint32_t W, std::uint32_t H, int frame) {
  static const std::uint8_t bars[8][3] = {
      {235, 235, 235}, {235, 235, 16}, {16, 235, 235}, {16, 235, 16},
      {235, 16, 235},  {235, 16, 16},  {16, 16, 235},  {16, 16, 16}};
  const std::uint32_t bx = (frame * 8u) % (W > 64 ? W - 64 : 1);
  const std::uint32_t by = H / 3;
  for (std::uint32_t y = 0; y < H; ++y) {
    for (std::uint32_t x = 0; x < W; ++x) {
      const std::uint8_t* c = bars[(x * 8 / W) & 7];
      std::uint8_t r = c[0], g = c[1], b = c[2];
      if (x >= bx && x < bx + 64 && y >= by && y < by + 64) {
        r = 255;
        g = 128;
        b = 0;  // moving orange box
      }
      std::uint8_t* d = p + (static_cast<size_t>(y) * W + x) * 4;
      d[0] = r;
      d[1] = g;
      d[2] = b;
      d[3] = 255;
    }
  }
}

// CPU reference RGB->NV12, limited-range BT.601, point-subsampled chroma -- the
// same math and siting the shader uses, so a correct GPU pack matches it.
void CpuRgbToNv12(const std::uint8_t* rgba, std::uint32_t W, std::uint32_t H,
                  std::uint32_t stride, std::vector<std::uint8_t>* out) {
  out->assign(static_cast<size_t>(stride) * H * 3 / 2, 0);
  auto Y = [](int r, int g, int b) {
    return std::clamp((257 * r + 504 * g + 98 * b + 16000) / 1000, 0, 255);
  };
  auto U = [](int r, int g, int b) {
    return std::clamp((-148 * r - 291 * g + 439 * b + 128000) / 1000, 0, 255);
  };
  auto V = [](int r, int g, int b) {
    return std::clamp((439 * r - 368 * g - 71 * b + 128000) / 1000, 0, 255);
  };
  std::uint8_t* y_plane = out->data();
  std::uint8_t* uv_plane = out->data() + static_cast<size_t>(stride) * H;
  for (std::uint32_t y = 0; y < H; ++y)
    for (std::uint32_t x = 0; x < W; ++x) {
      const std::uint8_t* s = rgba + (static_cast<size_t>(y) * W + x) * 4;
      y_plane[static_cast<size_t>(y) * stride + x] = Y(s[0], s[1], s[2]);
    }
  for (std::uint32_t cy = 0; cy < H / 2; ++cy)
    for (std::uint32_t cx = 0; cx < W / 2; ++cx) {
      // block center, matching the shader's ((cx*2+1)/W, (cy*2+1)/H).
      const std::uint32_t sx = std::min(cx * 2 + 1, W - 1);
      const std::uint32_t sy = std::min(cy * 2 + 1, H - 1);
      const std::uint8_t* s = rgba + (static_cast<size_t>(sy) * W + sx) * 4;
      std::uint8_t* d = uv_plane + static_cast<size_t>(cy) * stride + cx * 2;
      d[0] = U(s[0], s[1], s[2]);  // Cb
      d[1] = V(s[0], s[1], s[2]);  // Cr
    }
}

// ---- dma-heap (one contiguous NV12 buffer the encoder imports) -------------

int AllocDmabuf(size_t size) {
  const char* heaps[] = {"/dev/dma_heap/linux,cma",
                         "/dev/dma_heap/default_cma_region",
                         "/dev/dma_heap/system"};
  for (const char* path : heaps) {
    int heap = ::open(path, O_RDWR | O_CLOEXEC);
    if (heap < 0) continue;
    dma_heap_allocation_data a{};
    a.len = size;
    a.fd_flags = O_RDWR | O_CLOEXEC;
    int rc = ::ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &a);
    ::close(heap);
    if (rc == 0) return static_cast<int>(a.fd);
  }
  return -1;
}

void DmaSync(int fd, bool start, bool write) {
  dma_buf_sync s{};
  s.flags = (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) |
            (write ? DMA_BUF_SYNC_WRITE : DMA_BUF_SYNC_READ);
  ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

// ---- GL helpers ------------------------------------------------------------

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
    "#version 300 es\n"
    "in vec2 pos;\n"
    "void main(){ gl_Position = vec4(pos, 0.0, 1.0); }\n";

// Pack RGBA -> NV12 into a stride x (H*3/2) R8 target. gl_FragCoord.xy is the
// destination byte (column, row). Limited-range BT.601.
const char* kFrag =
    "#version 300 es\n"
    "precision highp float;\n"
    "uniform sampler2D src;\n"
    "uniform float uW, uH;\n"  // luma width/height in pixels
    "out vec4 frag;\n"
    "float luma(vec3 c){ return (0.257*c.r+0.504*c.g+0.098*c.b)+16.0/255.0; }\n"
    "float cb(vec3 c){ return (-0.148*c.r-0.291*c.g+0.439*c.b)+128.0/255.0; }\n"
    "float cr(vec3 c){ return (0.439*c.r-0.368*c.g-0.071*c.b)+128.0/255.0; }\n"
    "void main(){\n"
    "  float x = floor(gl_FragCoord.x);\n"
    "  float y = floor(gl_FragCoord.y);\n"
    "  float outv;\n"
    "  if (y < uH) {\n"
    "    vec3 c = texture(src, vec2((x+0.5)/uW, (y+0.5)/uH)).rgb;\n"
    "    outv = luma(c);\n"
    "  } else {\n"
    "    float cyf = y - uH;\n"
    "    float cxf = floor(x*0.5);\n"
    "    vec3 c = texture(src, vec2((cxf*2.0+1.0)/uW, (cyf*2.0+1.0)/uH)).rgb;\n"
    "    outv = (mod(x,2.0) < 1.0) ? cb(c) : cr(c);\n"  // even=Cb, odd=Cr
    "  }\n"
    "  frag = vec4(outv, 0.0, 0.0, 1.0);\n"
    "}\n";

struct Ext {
  PFNEGLCREATEIMAGEKHRPROC create = nullptr;
  PFNEGLDESTROYIMAGEKHRPROC destroy = nullptr;
  PFNGLEGLIMAGETARGETTEXTURE2DOESPROC target = nullptr;
};

bool HasStartCode(const std::vector<std::uint8_t>& au) {
  for (size_t i = 0; i + 2 < au.size(); ++i)
    if (au[i] == 0 && au[i + 1] == 0 && au[i + 2] == 1) return true;
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  const std::uint32_t W = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 640;
  const std::uint32_t H = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 480;
  const char* enc_dev = argc > 3 ? argv[3] : "/dev/video11";
  const char* render = argc > 4 ? argv[4] : "/dev/dri/renderD128";
  const int kFrames = 30;
  const std::uint32_t stride =
      W;  // 640 packs tightly on bcm2835; keep it simple
  const std::uint32_t th = H * 3 / 2;
  const size_t nv12_size = static_cast<size_t>(stride) * H * 3 / 2;

  // --- headless EGL (ES3) on the render node ---
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
  EGLDisplay dpy = get_plat ? get_plat(EGL_PLATFORM_GBM_KHR, gbm, nullptr)
                            : eglGetDisplay(EGL_DEFAULT_DISPLAY);
  EGLint major = 0, minor = 0;
  if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor)) {
    LOGE("eglInitialize failed\n");
    return 1;
  }
  const char* exts = eglQueryString(dpy, EGL_EXTENSIONS);
  if (!exts || !std::strstr(exts, "EGL_EXT_image_dma_buf_import")) {
    LOGE(
        "no EGL_EXT_image_dma_buf_import; cannot render into a dma-buf here\n");
    return 1;
  }
  eglBindAPI(EGL_OPENGL_ES_API);
  // We render only to an FBO under a surfaceless context, so the config's
  // surface type is immaterial -- GBM/Mesa exposes window-type configs, so ask
  // for one (as pi_hevc_egl does) rather than a pbuffer it may not offer.
  const EGLint cfg_attrs[] = {EGL_SURFACE_TYPE,
                              EGL_WINDOW_BIT,
                              EGL_RENDERABLE_TYPE,
                              EGL_OPENGL_ES3_BIT,
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
  const EGLint ctx_attrs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
  EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attrs);
  if (ctx == EGL_NO_CONTEXT ||
      !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
    LOGE("eglCreateContext/MakeCurrent failed: 0x%x\n", eglGetError());
    return 1;
  }
  LOGE("GL_RENDERER: %s\n", glGetString(GL_RENDERER));

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

  // --- the NV12 dma-buf the encoder will import, mapped for the CPU check ---
  int nv12_fd = AllocDmabuf(nv12_size);
  if (nv12_fd < 0) {
    LOGE("dma-heap alloc of %zu bytes failed\n", nv12_size);
    return 1;
  }
  void* nv12_map = ::mmap(nullptr, nv12_size, PROT_READ | PROT_WRITE,
                          MAP_SHARED, nv12_fd, 0);
  if (nv12_map == MAP_FAILED) {
    LOGE("mmap of NV12 dma-buf failed\n");
    return 1;
  }

  // Import it as one tall R8 image and try to hang an FBO on it (zero-copy).
  const EGLint img_attrs[] = {EGL_WIDTH,
                              static_cast<EGLint>(stride),
                              EGL_HEIGHT,
                              static_cast<EGLint>(th),
                              EGL_LINUX_DRM_FOURCC_EXT,
                              static_cast<EGLint>(kR8),
                              EGL_DMA_BUF_PLANE0_FD_EXT,
                              nv12_fd,
                              EGL_DMA_BUF_PLANE0_OFFSET_EXT,
                              0,
                              EGL_DMA_BUF_PLANE0_PITCH_EXT,
                              static_cast<EGLint>(stride),
                              EGL_NONE};
  EGLImageKHR image = ext.create(dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT,
                                 nullptr, img_attrs);

  GLuint dmabuf_tex = 0, dmabuf_fbo = 0;
  bool zero_copy = false;
  if (image != EGL_NO_IMAGE_KHR) {
    glGenTextures(1, &dmabuf_tex);
    glBindTexture(GL_TEXTURE_2D, dmabuf_tex);
    ext.target(GL_TEXTURE_2D, static_cast<GLeglImageOES>(image));
    glGenFramebuffers(1, &dmabuf_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, dmabuf_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           dmabuf_tex, 0);
    zero_copy =
        glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  }
  LOGE("render target: %s\n",
       zero_copy ? "zero-copy (FBO on the imported dma-buf)"
                 : "fallback (offscreen R8 + glReadPixels into the dma-buf)");

  // Fallback target: a plain R8 texture we glReadPixels out of.
  GLuint scratch_tex = 0, scratch_fbo = 0;
  if (!zero_copy) {
    glGenTextures(1, &scratch_tex);
    glBindTexture(GL_TEXTURE_2D, scratch_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, stride, th, 0, GL_RED,
                 GL_UNSIGNED_BYTE, nullptr);
    glGenFramebuffers(1, &scratch_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, scratch_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           scratch_tex, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      LOGE("fallback R8 FBO also incomplete; cannot render NV12\n");
      return 1;
    }
  }

  // --- program + source texture ---
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

  GLuint src_tex = 0;
  glGenTextures(1, &src_tex);
  glBindTexture(GL_TEXTURE_2D, src_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // --- the encoder (optional: the GPU pack is validated on its own by the CPU
  // cross-check, so a box without the hardware encoder still exercises and
  // verifies the RGBA->NV12 stage; only the encode step is skipped) ---
  auto enc = V4l2M2mEncoder::Create(enc_dev, W, H, 4'000'000, 30, 30);
  if (!enc) {
    LOGE(
        "encoder %s unavailable; validating the GPU pack only, skipping "
        "encode\n",
        enc_dev);
  }

  std::vector<std::uint8_t> rgba(static_cast<size_t>(W) * H * 4);
  std::vector<std::uint8_t> readback;
  if (!zero_copy) readback.resize(static_cast<size_t>(stride) * th);
  int frames_out = 0, cpu_max_diff = -1;
  double cpu_mean_diff = 0;
  size_t total = 0;
  bool got_key = false, start_code_ok = false;

  int fds[2] = {nv12_fd, nv12_fd};
  std::uint32_t offsets[2] = {0, stride * H};
  std::uint32_t strides[2] = {stride, stride};

  for (int f = 0; f < kFrames; ++f) {
    // "Flutter" renders a frame: upload the RGBA pattern as the source texture.
    FillRgba(rgba.data(), W, H, f);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, src_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 rgba.data());

    // RGBA -> NV12 pack.
    glBindFramebuffer(GL_FRAMEBUFFER, zero_copy ? dmabuf_fbo : scratch_fbo);
    glViewport(0, 0, stride, th);
    glUseProgram(prog);
    glUniform1i(glGetUniformLocation(prog, "src"), 0);
    glUniform1f(glGetUniformLocation(prog, "uW"), static_cast<float>(W));
    glUniform1f(glGetUniformLocation(prog, "uH"), static_cast<float>(H));
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    if (zero_copy) DmaSync(nv12_fd, /*start=*/true, /*write=*/true);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    if (zero_copy) {
      glFinish();
      DmaSync(nv12_fd, /*start=*/false, /*write=*/true);
    } else {
      // Read the packed R8 target straight into the dma-buf (one CPU copy).
      glReadPixels(0, 0, stride, th, GL_RED, GL_UNSIGNED_BYTE, readback.data());
      DmaSync(nv12_fd, true, true);
      std::memcpy(nv12_map, readback.data(), nv12_size);
      DmaSync(nv12_fd, false, true);
    }
    if (glGetError() != GL_NO_ERROR) {
      LOGE("GL error during pack at frame %d\n", f);
      return 1;
    }

    // CPU cross-check of the packed NV12 against a CPU reference (frame 0).
    if (f == 0) {
      std::vector<std::uint8_t> ref;
      CpuRgbToNv12(rgba.data(), W, H, stride, &ref);
      DmaSync(nv12_fd, true, false);
      const auto* got = static_cast<const std::uint8_t*>(nv12_map);
      long acc = 0;
      int worst = 0;
      for (size_t i = 0; i < nv12_size; ++i) {
        int d = std::abs(static_cast<int>(got[i]) - ref[i]);
        acc += d;
        worst = std::max(worst, d);
      }
      DmaSync(nv12_fd, false, false);
      cpu_mean_diff = static_cast<double>(acc) / nv12_size;
      cpu_max_diff = worst;
    }

    // Encode the NV12 dma-buf zero-copy (when the hardware encoder is present).
    if (enc) {
      std::vector<std::uint8_t> au;
      bool kf = false;
      if (!enc->EncodeDmabuf(fds, offsets, strides, 2,
                             static_cast<std::uint64_t>(f) * 33333, f == 0, &au,
                             &kf)) {
        LOGE("EncodeDmabuf frame %d failed\n", f);
        return 1;
      }
      if (!au.empty()) {
        if (frames_out == 0) start_code_ok = HasStartCode(au);
        ++frames_out;
        total += au.size();
        got_key = got_key || kf;
      }
    }
  }

  const bool cpu_ok =
      cpu_max_diff >= 0 && cpu_mean_diff < 2.0 && cpu_max_diff < 8;
  const bool enc_ran = static_cast<bool>(enc);
  const bool enc_ok = frames_out > 0 && got_key && start_code_ok;
  LOGE("----\n");
  LOGE("GPU pack vs CPU reference: mean|d|=%.3f worst=%d -> %s\n",
       cpu_mean_diff, cpu_max_diff, cpu_ok ? "MATCH" : "MISMATCH");
  if (enc_ran)
    LOGE("encoder: frames_out=%d bytes=%zu keyframe=%d start_code=%d -> %s\n",
         frames_out, total, got_key, start_code_ok, enc_ok ? "OK" : "FAIL");
  else
    LOGE("encoder: SKIPPED (no hardware encoder on this device)\n");
  LOGE("=== render=%s  pack=%s  encode=%s ===\n",
       zero_copy ? "zero-copy" : "fallback-readback", cpu_ok ? "PASS" : "FAIL",
       enc_ran ? (enc_ok ? "PASS" : "FAIL") : "SKIP");

  if (image != EGL_NO_IMAGE_KHR) ext.destroy(dpy, image);
  ::munmap(nv12_map, nv12_size);
  ::close(nv12_fd);
  eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  eglDestroyContext(dpy, ctx);
  eglTerminate(dpy);
  gbm_device_destroy(gbm);
  ::close(rfd);
  // Fail on a bad GPU pack or a failed encode; a skipped encode (wrong board)
  // is a partial pass (exit 2) so the GPU result is still visible in CI.
  if (!cpu_ok || (enc_ran && !enc_ok)) return 1;
  return enc_ran ? 0 : 2;
}

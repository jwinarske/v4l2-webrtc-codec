// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Pixel-exact on-hardware validation for VaapiH265Decoder. It decodes a HEVC
// clip with the real decoder, imports each exported dma-buf (with its tiling
// modifier) into a private VA display, detiles it to a linear NV12 image with
// vaGetImage -- exactly what a dma-buf consumer does -- and compares every
// luma and chroma sample to a software (ffmpeg) reference. HEVC decoding is
// bit-exact, so a conformant hardware decoder must match the reference
// sample-for-sample.
//
// The clip must have no reordering (encode with bframes=0) so the decoder's
// decode-order output lines up frame-for-frame with the display-order
// reference. Build the reference with:
//   ffmpeg -i clip.hevc -f rawvideo -pix_fmt yuv420p ref.yuv
//
// This needs a VAAPI device with the clip's profile, so it is opt-in
// (-DV4L2WC_ENABLE_HW_TESTS=ON) and not registered with ctest. It links libva
// directly (only this hardware test does; the decode core still dlopens it).
// Validated on an AMD Radeon (radeonsi, VA-API 1.23): a libx265 Main 8-bit
// 4:2:0 clip matches the reference with a worst delta of 0 across all frames.
//
// Usage: vaapi_h265_pixel_test <clip.hevc> <ref.yuv> [render_node]

#include <fcntl.h>
#include <unistd.h>
#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "src/dma_decoder.h"
#include "src/vaapi_h265_decoder.h"

namespace {

std::vector<std::uint8_t> ReadFile(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return {};
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<std::uint8_t> b(n > 0 ? n : 0);
  if (n > 0) {
    size_t r = fread(b.data(), 1, static_cast<size_t>(n), f);
    (void)r;
  }
  fclose(f);
  return b;
}

std::vector<std::vector<std::uint8_t>> SplitAccessUnits(
    const std::vector<std::uint8_t>& d) {
  std::vector<std::size_t> st;
  for (std::size_t j = 0; j + 3 <= d.size(); ++j)
    if (d[j] == 0 && d[j + 1] == 0 && d[j + 2] == 1) st.push_back(j);
  std::vector<std::vector<std::uint8_t>> aus;
  std::vector<std::uint8_t> cur;
  bool vcl = false;
  for (std::size_t k = 0; k < st.size(); ++k) {
    std::size_t s = st[k], e = (k + 1 < st.size()) ? st[k + 1] : d.size();
    bool is_vcl = ((d[s + 3] >> 1) & 0x3f) <= 31;
    if (is_vcl && vcl) {
      aus.push_back(cur);
      cur.clear();
      vcl = false;
    }
    cur.insert(cur.end(), d.begin() + s, d.begin() + e);
    if (is_vcl) vcl = true;
  }
  if (!cur.empty()) aus.push_back(cur);
  return aus;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::printf("usage: %s <clip.hevc> <ref.yuv> [render_node]\n", argv[0]);
    return 2;
  }
  const char* node = argc >= 4 ? argv[3] : "/dev/dri/renderD128";
  auto aus = SplitAccessUnits(ReadFile(argv[1]));
  auto ref = ReadFile(argv[2]);
  if (aus.empty() || ref.empty()) {
    std::printf("FAIL: empty clip or reference\n");
    return 1;
  }

  int drm = open(node, O_RDWR | O_CLOEXEC);
  VADisplay dpy = vaGetDisplayDRM(drm);
  int maj = 0, minr = 0;
  if (vaInitialize(dpy, &maj, &minr) != VA_STATUS_SUCCESS) {
    std::printf("FAIL: vaInitialize(%s)\n", node);
    return 1;
  }
  auto dec = v4l2wc::VaapiH265Decoder::Create(node, 16);
  if (!dec) {
    std::printf("FAIL: VaapiH265Decoder::Create(%s)\n", node);
    return 1;
  }

  int frames = 0;
  int mismatched = 0;
  int worst = 0;
  std::uint64_t ts = 0;
  for (auto& au : aus) {
    if (dec->SubmitBitstream(au.data(), au.size(), ts++) !=
        v4l2wc::SubmitResult::kOk)
      continue;
    dec->Drive();
    v4l2wc::V4l2DmaFrame f;
    if (!dec->Acquire(&f)) continue;
    const std::uint32_t w = f.width, h = f.height;

    // Import the exported dma-buf (fourcc + tiling modifier + plane layout) and
    // detile to a linear NV12 image.
    VADRMPRIMESurfaceDescriptor d{};
    d.fourcc = VA_FOURCC_NV12;
    d.width = w;
    d.height = h;
    d.num_objects = 1;
    d.objects[0].fd = f.fds[0];
    d.objects[0].size =
        static_cast<std::uint32_t>(lseek(f.fds[0], 0, SEEK_END));
    d.objects[0].drm_format_modifier = f.modifier;
    d.num_layers = 1;
    d.layers[0].drm_format = f.drm_fourcc;
    d.layers[0].num_planes = f.num_planes;
    for (std::uint32_t p = 0; p < f.num_planes; ++p) {
      d.layers[0].object_index[p] = 0;
      d.layers[0].offset[p] = f.offsets[p];
      d.layers[0].pitch[p] = f.pitches[p];
    }
    VASurfaceAttrib attrs[2]{};
    attrs[0].type = VASurfaceAttribMemoryType;
    attrs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrs[0].value.type = VAGenericValueTypeInteger;
    attrs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
    attrs[1].type = VASurfaceAttribExternalBufferDescriptor;
    attrs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrs[1].value.type = VAGenericValueTypePointer;
    attrs[1].value.value.p = &d;
    VASurfaceID surf = VA_INVALID_SURFACE;
    if (vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, &surf, 1, attrs, 2) !=
        VA_STATUS_SUCCESS) {
      std::printf("FAIL frame %d: dma-buf import failed\n", frames);
      return 1;
    }
    VAImageFormat fmt{};
    fmt.fourcc = VA_FOURCC_NV12;
    fmt.byte_order = VA_LSB_FIRST;
    fmt.bits_per_pixel = 12;
    VAImage img{};
    void* map = nullptr;
    VAStatus s = vaCreateImage(dpy, &fmt, w, h, &img);
    if (s == VA_STATUS_SUCCESS)
      s = vaGetImage(dpy, surf, 0, 0, w, h, img.image_id);
    if (s == VA_STATUS_SUCCESS) s = vaMapBuffer(dpy, img.buf, &map);
    if (s != VA_STATUS_SUCCESS) {
      std::printf("FAIL frame %d: detile (%s)\n", frames, vaErrorStr(s));
      return 1;
    }
    const std::uint8_t* base = static_cast<const std::uint8_t*>(map);
    const std::uint8_t* iy = base + img.offsets[0];
    const std::uint8_t* iuv = base + img.offsets[1];

    // Reference planes for this frame (I420: Y, then U, then V).
    const std::size_t fsize = static_cast<std::size_t>(w) * h * 3 / 2;
    const std::uint8_t* ry =
        ref.data() + static_cast<std::size_t>(frames) * fsize;
    const std::uint8_t* ru = ry + static_cast<std::size_t>(w) * h;
    const std::uint8_t* rv = ru + (w / 2) * (h / 2);

    long long diffs = 0;
    for (std::uint32_t y = 0; y < h; ++y)
      for (std::uint32_t x = 0; x < w; ++x) {
        int dd = std::abs(iy[y * img.pitches[0] + x] - ry[y * w + x]);
        if (dd) {
          ++diffs;
          if (dd > worst) worst = dd;
        }
      }
    for (std::uint32_t y = 0; y < h / 2; ++y)
      for (std::uint32_t x = 0; x < w / 2; ++x) {
        int du =
            std::abs(iuv[y * img.pitches[1] + 2 * x] - ru[y * (w / 2) + x]);
        int dv =
            std::abs(iuv[y * img.pitches[1] + 2 * x + 1] - rv[y * (w / 2) + x]);
        if (du || dv) ++diffs;
        if (du > worst) worst = du;
        if (dv > worst) worst = dv;
      }
    if (diffs) {
      ++mismatched;
      std::printf("  frame %2d: %lld differing samples\n", frames, diffs);
    }

    vaUnmapBuffer(dpy, img.buf);
    vaDestroyImage(dpy, img.image_id);
    vaDestroySurfaces(dpy, &surf, 1);
    dec->Release(f.capture_index);
    ++frames;
  }

  std::printf("RESULT: %d frames, %d mismatched, worst sample delta %d\n",
              frames, mismatched, worst);
  if (frames == 0 || mismatched != 0) {
    std::printf("FAIL\n");
    return 1;
  }
  std::printf("vaapi h265 pixel test: ok (bit-exact)\n");
  return 0;
}

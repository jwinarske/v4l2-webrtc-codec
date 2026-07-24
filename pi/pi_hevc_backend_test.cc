// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-device test for V4l2StatelessH265Decoder: drives the real IDmaDecoder
// surface (SubmitBitstream / Acquire / Release) one access unit at a time,
// maps each Acquired dma-buf, detiles the Broadcom NV12_COL128 output into
// planar I420, and writes it out for a byte-exact compare against an ffmpeg
// software reference. Cross-built for aarch64 with emb; run on the Pi.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h265/nal.h"
#include "src/v4l2_stateless_h265_decoder.h"

using namespace v4l2wc;

namespace {

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

// Splits an Annex-B stream into access units: a new AU starts at the first
// non-VCL NAL that follows a VCL, or at a first-slice VCL following a VCL.
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

// Detiles one NV12_COL128 plane (128-byte columns) into raster.
void Detile(const std::uint8_t* t, std::uint32_t w_bytes, std::uint32_t h,
            std::uint32_t col_h, std::vector<std::uint8_t>* o) {
  o->resize(static_cast<size_t>(w_bytes) * h);
  for (std::uint32_t y = 0; y < h; ++y)
    for (std::uint32_t bx = 0; bx < w_bytes; ++bx)
      (*o)[static_cast<size_t>(y) * w_bytes + bx] =
          t[(bx / 128) * 128 * col_h + y * 128 + (bx % 128)];
}

// Unpacks a Broadcom SAND 10-bit (Nc30) row: each little-endian 32-bit word
// holds three 10-bit samples (bits [0:9], [10:19], [20:29]). Emits `n` 16-bit
// little-endian samples, matching ffmpeg yuv420p10le.
void Unpack10(const std::uint8_t* row, std::uint32_t n, std::uint8_t* out16) {
  std::uint32_t s = 0;
  for (std::uint32_t g = 0; s < n; ++g) {
    const std::uint32_t word =
        static_cast<std::uint32_t>(row[g * 4]) |
        (static_cast<std::uint32_t>(row[g * 4 + 1]) << 8) |
        (static_cast<std::uint32_t>(row[g * 4 + 2]) << 16) |
        (static_cast<std::uint32_t>(row[g * 4 + 3]) << 24);
    for (int k = 0; k < 3 && s < n; ++k, ++s) {
      const std::uint32_t v = (word >> (10 * k)) & 0x3ff;
      out16[2 * s] = static_cast<std::uint8_t>(v & 0xff);
      out16[2 * s + 1] = static_cast<std::uint8_t>((v >> 8) & 0xff);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s <clip.hevc> <out.yuv> [node=/dev/video19]\n",
                 argv[0]);
    return 2;
  }
  const char* node = argc > 3 ? argv[3] : "/dev/video19";
  std::vector<std::uint8_t> data = ReadFile(argv[1]);
  if (data.empty()) {
    std::fprintf(stderr, "cannot read %s\n", argv[1]);
    return 1;
  }
  auto dec = V4l2StatelessH265Decoder::Create(node, 16);
  if (!dec) {
    std::fprintf(stderr, "decoder create failed\n");
    return 1;
  }
  FILE* fout = std::fopen(argv[2], "wb");
  auto aus = SplitAccessUnits(data);
  std::fprintf(stderr, "%zu access units\n", aus.size());

  int frames = 0;
  std::uint64_t ts = 0;
  auto pump = [&](V4l2DmaFrame& f) {
    const std::uint32_t w = f.width, h = f.height;
    struct stat st0{};
    struct stat st1{};
    fstat(f.fds[0], &st0);
    fstat(f.fds[1], &st1);
    auto* y = static_cast<std::uint8_t*>(
        mmap(nullptr, st0.st_size, PROT_READ, MAP_SHARED, f.fds[0], 0));
    auto* uv = static_cast<std::uint8_t*>(
        mmap(nullptr, st1.st_size, PROT_READ, MAP_SHARED, f.fds[1], 0));
    if (y != MAP_FAILED && uv != MAP_FAILED) {
      // The tiled column height is the format's plane height (luma) and half
      // (chroma); production code would read it from the SAND DRM modifier.
      const std::uint32_t stride = f.pitches[0];
      const bool ten = f.drm_fourcc == v4l2_fourcc('N', 'c', '3', '0');
      std::vector<std::uint8_t> yout, uout, vout;
      if (ten) {
        // Packed SAND 10-bit: detile the packed bytes, then unpack 3-per-4 into
        // 16-bit samples; chroma unpacks the interleaved UV stream (w samples
        // per row) and de-interleaves it.
        std::vector<std::uint8_t> yb, cb;
        Detile(y, stride, h, h, &yb);
        Detile(uv, stride, h / 2, h / 2, &cb);
        yout.resize(static_cast<size_t>(w) * h * 2);
        for (std::uint32_t r = 0; r < h; ++r)
          Unpack10(&yb[static_cast<size_t>(r) * stride], w,
                   &yout[static_cast<size_t>(r) * w * 2]);
        uout.resize((w / 2) * (h / 2) * 2);
        vout.resize((w / 2) * (h / 2) * 2);
        std::vector<std::uint8_t> crow(static_cast<size_t>(w) * 2);
        for (std::uint32_t cy = 0; cy < h / 2; ++cy) {
          Unpack10(&cb[static_cast<size_t>(cy) * stride], w, crow.data());
          for (std::uint32_t cx = 0; cx < w / 2; ++cx) {
            const size_t di = (static_cast<size_t>(cy) * (w / 2) + cx) * 2;
            uout[di] = crow[(2 * cx) * 2];
            uout[di + 1] = crow[(2 * cx) * 2 + 1];
            vout[di] = crow[(2 * cx + 1) * 2];
            vout[di + 1] = crow[(2 * cx + 1) * 2 + 1];
          }
        }
      } else {
        std::vector<std::uint8_t> uvp;
        Detile(y, w, h, h, &yout);
        Detile(uv, w, h / 2, h / 2, &uvp);
        uout.resize((w / 2) * (h / 2));
        vout.resize((w / 2) * (h / 2));
        for (std::uint32_t cy = 0; cy < h / 2; ++cy)
          for (std::uint32_t cx = 0; cx < w / 2; ++cx) {
            uout[cy * (w / 2) + cx] = uvp[cy * w + 2 * cx];
            vout[cy * (w / 2) + cx] = uvp[cy * w + 2 * cx + 1];
          }
      }
      if (fout) {
        std::fwrite(yout.data(), 1, yout.size(), fout);
        std::fwrite(uout.data(), 1, uout.size(), fout);
        std::fwrite(vout.data(), 1, vout.size(), fout);
      }
      ++frames;
    }
    if (y != MAP_FAILED) munmap(y, st0.st_size);
    if (uv != MAP_FAILED) munmap(uv, st1.st_size);
    dec->Release(f.capture_index);
  };

  for (auto& au : aus) {
    dec->SubmitBitstream(au.data(), au.size(), ts++);
    V4l2DmaFrame f;
    while (dec->Acquire(&f)) pump(f);
  }
  // End of stream: flush the reorder buffer and drain the remaining frames.
  dec->Drain();
  V4l2DmaFrame f;
  while (dec->Acquire(&f)) pump(f);
  if (fout) std::fclose(fout);
  std::fprintf(stderr, "=== decoded %d frames ===\n", frames);
  return frames > 0 ? 0 : 1;
}

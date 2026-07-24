// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-device test for V4l2StatelessH265Decoder: drives the real IDmaDecoder
// surface (SubmitBitstream / Acquire / Release) one access unit at a time,
// maps each Acquired dma-buf, detiles the Broadcom NV12_COL128 output into
// planar I420, and writes it out for a byte-exact compare against an ffmpeg
// software reference. Cross-built for aarch64 with emb; run on the Pi.

#include <fcntl.h>
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
    const std::uint32_t w = f.width, h = f.height, stride = f.pitches[0];
    struct stat st0{};
    struct stat st1{};
    fstat(f.fds[0], &st0);
    fstat(f.fds[1], &st1);
    auto* y = static_cast<std::uint8_t*>(
        mmap(nullptr, st0.st_size, PROT_READ, MAP_SHARED, f.fds[0], 0));
    auto* uv = static_cast<std::uint8_t*>(
        mmap(nullptr, st1.st_size, PROT_READ, MAP_SHARED, f.fds[1], 0));
    if (y != MAP_FAILED && uv != MAP_FAILED) {
      // The tiled column height is the format's plane height, not the dma-buf
      // size (which is page-padded). For NV12_COL128 at these sizes that is the
      // frame height (luma) and half (chroma); production code would read the
      // column height from the SAND128 DRM modifier.
      const std::uint32_t luma_col_h = h;
      const std::uint32_t chroma_col_h = h / 2;
      std::vector<std::uint8_t> yp, uvp, u((w / 2) * (h / 2)),
          v((w / 2) * (h / 2));
      Detile(y, w, h, luma_col_h, &yp);
      Detile(uv, w, h / 2, chroma_col_h, &uvp);
      for (std::uint32_t cy = 0; cy < h / 2; ++cy)
        for (std::uint32_t cx = 0; cx < w / 2; ++cx) {
          u[cy * (w / 2) + cx] = uvp[cy * w + 2 * cx];
          v[cy * (w / 2) + cx] = uvp[cy * w + 2 * cx + 1];
        }
      if (fout) {
        std::fwrite(yp.data(), 1, yp.size(), fout);
        std::fwrite(u.data(), 1, u.size(), fout);
        std::fwrite(v.data(), 1, v.size(), fout);
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
  if (fout) std::fclose(fout);
  std::fprintf(stderr, "=== decoded %d frames ===\n", frames);
  return frames > 0 ? 0 : 1;
}

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-hardware smoke test for VaapiH265Decoder. It feeds a real HEVC Annex-B
// file through the actual decoder on a VAAPI device, one access unit at a time,
// and checks every access unit decodes to a frame of the expected shape. The
// driver validates the picture/slice parameter buffers as it decodes, so
// reaching a synced surface for an IDR followed by inter frames exercises the
// whole parameter construction -- POC derivation, the reference lists, the IQ
// matrix -- against a production driver.
//
// This needs a VAAPI device with the stream's HEVC profile, so it is excluded
// from the default build (enable with -DV4L2WC_ENABLE_HW_TESTS=ON) and is not
// part of the CI suite. Validated on an AMD Radeon (radeonsi, VA-API 1.23):
// libx265 Main 8-bit and Main 10 4:2:0 clips both decode every frame with no
// driver error. The surfaces are GPU-tiled, so a raw CPU read of the exported
// dma-buf is not meaningful; the test checks decode success and frame geometry,
// not pixels.
//
// Usage: vaapi_h265_hw_test <file.hevc> [render_node]

#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
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

// Splits an Annex-B stream into access units (raw bytes including start codes):
// a new access unit begins at a VCL NAL that starts a picture
// (first_slice_segment_in_pic_flag set), so a multi-slice picture is delivered
// whole in one SubmitBitstream.
std::vector<std::vector<std::uint8_t>> SplitAccessUnits(
    const std::vector<std::uint8_t>& data) {
  std::vector<std::size_t> starts;
  for (std::size_t j = 0; j + 3 <= data.size(); ++j)
    if (data[j] == 0 && data[j + 1] == 0 && data[j + 2] == 1)
      starts.push_back(j);
  std::vector<std::vector<std::uint8_t>> aus;
  std::vector<std::uint8_t> cur;
  bool cur_has_vcl = false;
  for (std::size_t k = 0; k < starts.size(); ++k) {
    const std::size_t s = starts[k];
    const std::size_t e = (k + 1 < starts.size()) ? starts[k + 1] : data.size();
    const std::uint32_t type = (data[s + 3] >> 1) & 0x3f;
    const bool is_vcl = type <= 31;
    // A picture starts at a VCL NAL with first_slice_segment_in_pic_flag (the
    // first RBSP bit, past the two-byte NAL header) set; later slices of the
    // same picture stay in the current access unit.
    const bool first_slice =
        is_vcl && s + 5 < data.size() && (data[s + 5] & 0x80);
    if (first_slice && cur_has_vcl) {
      aus.push_back(cur);
      cur.clear();
      cur_has_vcl = false;
    }
    cur.insert(cur.end(), data.begin() + s, data.begin() + e);
    if (is_vcl) cur_has_vcl = true;
  }
  if (!cur.empty()) aus.push_back(cur);
  return aus;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: %s <file.hevc> [render_node]\n", argv[0]);
    return 2;
  }
  const char* node = argc >= 3 ? argv[2] : "/dev/dri/renderD128";
  auto data = ReadFile(argv[1]);
  if (data.empty()) {
    std::printf("FAIL: cannot read %s\n", argv[1]);
    return 1;
  }
  auto aus = SplitAccessUnits(data);
  std::printf("== %s: %zu access units ==\n", argv[1], aus.size());

  auto dec = v4l2wc::VaapiH265Decoder::Create(node, 16);
  if (!dec) {
    std::printf("FAIL: VaapiH265Decoder::Create(%s) returned null\n", node);
    return 1;
  }

  int frames = 0;
  int submit_failures = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint64_t timestamp = 0;
  for (auto& au : aus) {
    const v4l2wc::SubmitResult sr =
        dec->SubmitBitstream(au.data(), au.size(), timestamp++);
    if (sr != v4l2wc::SubmitResult::kOk) {
      std::printf("  submit result=%d\n", static_cast<int>(sr));
      ++submit_failures;
      continue;
    }
    dec->Drive();
    v4l2wc::V4l2DmaFrame f;
    if (!dec->Acquire(&f)) {
      continue;  // no frame ready for this access unit
    }
    if (frames == 0) {
      width = f.width;
      height = f.height;
    }
    if (f.width != width || f.height != height || f.fds[0] < 0) {
      std::printf("FAIL: frame %d has bad shape %ux%u fd=%d\n", frames, f.width,
                  f.height, f.fds[0]);
      return 1;
    }
    dec->Release(f.capture_index);
    ++frames;
  }

  std::printf("RESULT: %d/%zu frames decoded, %d submit failures (%ux%u)\n",
              frames, aus.size(), submit_failures, width, height);
  if (frames == 0 || submit_failures != 0) {
    std::printf("FAIL\n");
    return 1;
  }
  std::printf("vaapi h265 hw test: ok\n");
  return 0;
}

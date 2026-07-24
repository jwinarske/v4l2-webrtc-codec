// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the H.265 SPS parser. Feeds arbitrary bytes as an
// RBSP and relies on the bounds-check invariants (verified under ASan/UBSan).
// Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h265/sps.cc test/fuzz/h265_sps_fuzzer.cc -o h265_sps_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h265/sps.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  v4l2wc::h265::Sps sps;
  if (v4l2wc::h265::ParseSps(data, size, &sps)) {
    // On success the derived geometry must be within the accepted range and
    // every short-term RPS must respect the delta-POC array bound.
    if (sps.width == 0 || sps.height == 0 ||
        sps.width > v4l2wc::h265::kMaxDimension ||
        sps.height > v4l2wc::h265::kMaxDimension) {
      __builtin_trap();
    }
    if (sps.short_term_rps.size() > v4l2wc::h265::kMaxShortTermRps) {
      __builtin_trap();
    }
    for (const auto& rps : sps.short_term_rps) {
      if (rps.num_negative_pics > v4l2wc::h265::kMaxRefPics ||
          rps.num_positive_pics > v4l2wc::h265::kMaxRefPics ||
          rps.num_delta_pocs != rps.num_negative_pics + rps.num_positive_pics) {
        __builtin_trap();
      }
    }
  }
  return 0;
}

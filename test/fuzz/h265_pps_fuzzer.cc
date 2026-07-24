// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the H.265 PPS parser. Feeds arbitrary bytes as an
// RBSP and relies on the bounds-check invariants (verified under ASan/UBSan).
// Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h265/pps.cc test/fuzz/h265_pps_fuzzer.cc -o h265_pps_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h265/pps.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  v4l2wc::h265::Pps pps;
  if (v4l2wc::h265::ParsePps(data, size, &pps)) {
    // On success the loop-bounding counts must be within the accepted range.
    if (pps.num_ref_idx_l0_default_active_minus1 >
            v4l2wc::h265::kMaxRefIdxDefault ||
        pps.num_ref_idx_l1_default_active_minus1 >
            v4l2wc::h265::kMaxRefIdxDefault ||
        pps.num_tile_columns_minus1 >= v4l2wc::h265::kMaxTiles ||
        pps.num_tile_rows_minus1 >= v4l2wc::h265::kMaxTiles) {
      __builtin_trap();
    }
  }
  return 0;
}

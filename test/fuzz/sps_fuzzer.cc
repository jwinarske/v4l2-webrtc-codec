// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the SPS parser. Feeds arbitrary bytes as an RBSP and
// relies on the bounds-check invariants (verified under ASan/UBSan). Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h264/sps.cc test/fuzz/sps_fuzzer.cc -o sps_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h264/sps.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  v4l2wc::h264::Sps sps;
  if (v4l2wc::h264::ParseSps(data, size, &sps)) {
    // On success the derived dimensions must be within the accepted range.
    if (sps.width == 0 || sps.height == 0 ||
        sps.width > v4l2wc::h264::kMaxDimension ||
        sps.height > v4l2wc::h264::kMaxDimension) {
      __builtin_trap();
    }
  }
  return 0;
}

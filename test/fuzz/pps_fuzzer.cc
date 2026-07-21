// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the PPS parser. Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h264/pps.cc test/fuzz/pps_fuzzer.cc -o pps_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h264/pps.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  v4l2wc::h264::Pps pps;
  v4l2wc::h264::ParsePps(data, size, &pps);
  return 0;
}

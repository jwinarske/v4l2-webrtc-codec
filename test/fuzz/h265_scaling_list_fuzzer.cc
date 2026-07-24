// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the H.265 scaling_list_data parser. Feeds arbitrary
// bytes and relies on the bounds-check invariants (verified under ASan/UBSan).
// Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h265/scaling_list.cc test/fuzz/h265_scaling_list_fuzzer.cc \
//       -o h265_scaling_list_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/bit_reader.h"
#include "parse/h265/scaling_list.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  v4l2wc::BitReader br(data, size);
  v4l2wc::h265::ScalingListData out;
  ParseScalingListData(&br, &out);
  return 0;
}

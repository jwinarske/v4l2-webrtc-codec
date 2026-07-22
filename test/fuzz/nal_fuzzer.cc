// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the Annex-B NAL parser. The bitstream is
// attacker-controlled post-DTLS, so this feeds arbitrary bytes and relies on
// the parser's bounds-check invariants (verified under ASan/UBSan). Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h264/nal.cc test/fuzz/nal_fuzzer.cc -o nal_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h264/nal.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  auto nals = v4l2wc::h264::ParseAnnexB(data, size);
  // Touch the result so the call is not optimized away.
  size_t total = 0;
  for (const auto& nal : nals) {
    total += nal.rbsp.size();
    // The RBSP is derived from the raw NAL by removing bytes, so it can never
    // be longer, and any offset the conversion accepts must land inside raw.
    if (nal.rbsp.size() + 1 > nal.raw.size()) {
      __builtin_trap();
    }
    for (uint32_t bit :
         {0u, 1u, 7u, 8u, static_cast<uint32_t>(nal.rbsp.size() * 8)}) {
      uint32_t raw_bit = 0;
      if (v4l2wc::h264::RbspToRawBitOffset(nal, bit, &raw_bit) &&
          raw_bit > nal.raw.size() * 8) {
        __builtin_trap();
      }
    }
  }
  return total == SIZE_MAX ? 1 : 0;
}

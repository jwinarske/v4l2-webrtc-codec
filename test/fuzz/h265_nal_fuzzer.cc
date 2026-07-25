// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the H.265 Annex-B NAL parser. The bitstream is
// attacker-controlled post-DTLS, so this feeds arbitrary bytes and relies on
// the parser's bounds-check invariants (verified under ASan/UBSan). Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h265/nal.cc test/fuzz/h265_nal_fuzzer.cc -o h265_nal_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h265/nal.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  auto nals = v4l2wc::h265::ParseAnnexB(data, size);
  size_t total = 0;
  for (const auto& nal : nals) {
    total += nal.rbsp.size();
    // The RBSP is derived from the raw NAL by removing bytes, so it can never
    // be longer, and the two header bytes are always present.
    if (nal.raw.size() < 2 || nal.rbsp.size() + 2 > nal.raw.size()) {
      __builtin_trap();
    }
    for (uint32_t bit :
         {0u, 1u, 15u, 16u, static_cast<uint32_t>(nal.rbsp.size() * 8)}) {
      uint32_t raw_bit = 0;
      if (v4l2wc::h265::RbspToRawBitOffset(nal, bit, &raw_bit) &&
          raw_bit > nal.raw.size() * 8) {
        __builtin_trap();
      }
    }
  }
  return total == SIZE_MAX ? 1 : 0;
}

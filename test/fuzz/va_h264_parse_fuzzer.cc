// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the VA-oriented H.264 parser -- the VAAPI engine's
// bitstream front end, which sees attacker-controlled data after the DTLS
// handshake. Feeds arbitrary bytes as an Annex-B stream, splits it into NALs,
// and parses each as SPS / PPS / slice header, relying on the bounds-check
// invariants (verified under ASan/UBSan). Needs libva-devel for <va/va.h>.
// Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       src/va_h264_parse.cc test/fuzz/va_h264_parse_fuzzer.cc \
//       -o va_h264_parse_fuzzer

#include <cstddef>
#include <cstdint>
#include <vector>

#include "src/va_h264_parse.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  const std::vector<v4l2wc::va::Nal> nals = v4l2wc::va::SplitAnnexB(data, size);

  v4l2wc::va::Sps sps;
  v4l2wc::va::Pps pps;
  bool have_sps = false;
  bool have_pps = false;

  for (const v4l2wc::va::Nal& nal : nals) {
    // A split NAL must never claim more RBSP than the raw bytes it came from.
    if (nal.rbsp.size() > nal.raw.size()) {
      __builtin_trap();
    }
    switch (nal.type) {
      case 7:  // SPS
        have_sps = v4l2wc::va::ParseSps(nal.rbsp.data(), nal.rbsp.size(), &sps);
        break;
      case 8:  // PPS
        if (have_sps) {
          have_pps = v4l2wc::va::ParsePps(nal.rbsp.data(), nal.rbsp.size(),
                                          &sps, &pps);
        }
        break;
      case 1:  // non-IDR slice
      case 5:  // IDR slice
      {
        // Parse against whatever SPS/PPS we have; the defaults are
        // self-consistent, so the slice path stays reachable even when the
        // input carries no parameter sets. Without this the truncated-slice
        // paths are effectively unreachable from random input.
        (void)have_sps;
        (void)have_pps;
        v4l2wc::va::SliceHdr sh;
        if (v4l2wc::va::ParseSliceHdr(nal, sps, pps, &sh)) {
          // slice_data_bit_offset is handed to the hardware as a bit offset
          // into the raw NAL; it must lie within it.
          if (sh.data_bit_offset > nal.raw.size() * 8) {
            __builtin_trap();
          }
        }
      } break;
      default:
        break;
    }
  }
  return 0;
}

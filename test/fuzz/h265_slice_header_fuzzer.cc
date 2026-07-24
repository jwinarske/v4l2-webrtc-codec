// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the H.265 slice-segment header parser. The first bytes
// of the input parameterize the SliceContext (so the fuzzer explores IDR/IRAP,
// SAO, temporal-MVP, tiles, and short-term-RPS combinations) and the remainder
// is the slice RBSP. Relies on the bounds-check invariants (verified under
// ASan/UBSan). Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h265/nal.cc parse/h265/sps.cc parse/h265/slice_header.cc \
//       test/fuzz/h265_slice_header_fuzzer.cc -o h265_slice_header_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h265/nal.h"
#include "parse/h265/slice_header.h"
#include "parse/h265/sps.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  if (size < 4) {
    return 0;
  }
  // Derive a context from the first three bytes and one NAL type; the rest is
  // the RBSP under test.
  const uint8_t cfg0 = data[0];
  const uint8_t cfg1 = data[1];
  const uint8_t cfg2 = data[2];
  const auto nal_type = static_cast<v4l2wc::h265::NalUnitType>(data[3] & 0x3f);

  v4l2wc::h265::SliceContext ctx;
  ctx.pic_size_in_ctbs = (static_cast<uint32_t>(cfg0) << 4) | (cfg1 & 0x0f);
  ctx.log2_max_pic_order_cnt_lsb = 4 + (cfg1 >> 4) % 13;  // 4..16
  ctx.separate_colour_plane_flag = cfg2 & 1;
  ctx.chroma_array_type = (cfg2 >> 1) & 3;
  ctx.sample_adaptive_offset_enabled_flag = (cfg2 >> 3) & 1;
  ctx.sps_temporal_mvp_enabled_flag = (cfg2 >> 4) & 1;
  ctx.long_term_ref_pics_present_flag = (cfg2 >> 5) & 1;
  ctx.num_long_term_ref_pics_sps = (cfg2 >> 6) & 3;
  ctx.dependent_slice_segments_enabled_flag = (cfg0 >> 7) & 1;
  ctx.num_extra_slice_header_bits = (cfg1 >> 5) & 7;
  ctx.output_flag_present_flag = (cfg0 >> 6) & 1;
  // A couple of SPS-defined short-term RPSs, so the SPS-index and single-set
  // branches are reachable.
  v4l2wc::h265::ShortTermRps rps;
  rps.num_negative_pics = 1;
  rps.num_delta_pocs = 1;
  ctx.short_term_rps = {rps, rps};

  v4l2wc::h265::SliceHeader sh;
  if (v4l2wc::h265::ParseSliceHeader(data + 4, size - 4, nal_type, ctx, &sh)) {
    // On success the resolved active counts and the current RPS must respect
    // their bounds.
    if (sh.num_ref_idx_l0_active_minus1 > v4l2wc::h265::kMaxRefIdxDefault ||
        sh.num_ref_idx_l1_active_minus1 > v4l2wc::h265::kMaxRefIdxDefault ||
        sh.current_rps.num_delta_pocs > v4l2wc::h265::kMaxRefPics) {
      __builtin_trap();
    }
    if (!sh.first_slice_segment_in_pic_flag &&
        sh.slice_segment_address >= ctx.pic_size_in_ctbs) {
      __builtin_trap();
    }
  }
  return 0;
}

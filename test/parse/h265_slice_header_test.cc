// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.265 slice-segment header parser. The primary vectors are
// real slice headers from an ffmpeg/libx265 clip (a 1280x720 hierarchical-B
// GOP): the active SPS and PPS are parsed from the same stream to build the
// SliceContext, then an IDR, a P, and a B slice header are parsed and checked
// against the values ffmpeg produces. A synthetic dependent slice segment
// covers the early-return branch that the clip's slices (all independent) do
// not reach. No framework: a failing check sets the exit code.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h265/nal.h"
#include "parse/h265/pps.h"
#include "parse/h265/slice_header.h"
#include "parse/h265/sps.h"

using namespace v4l2wc::h265;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

// Wraps raw NAL bytes in a start code and returns the parsed NAL units.
static std::vector<Nal> WrapNal(const std::vector<uint8_t>& raw) {
  std::vector<uint8_t> buf = {0x00, 0x00, 0x01};
  buf.insert(buf.end(), raw.begin(), raw.end());
  return ParseAnnexB(buf.data(), buf.size());
}

// Minimal MSB-first bit writer for building a synthetic RBSP.
class BitWriter {
 public:
  void WriteBits(uint32_t value, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t bit = (value >> (n - 1 - i)) & 1u;
      if (bit_ == 0) {
        bytes_.push_back(0);
      }
      if (bit) {
        bytes_.back() |= static_cast<uint8_t>(1u << (7 - bit_));
      }
      bit_ = (bit_ + 1) & 7;
    }
  }
  void WriteFlag(bool v) { WriteBits(v ? 1 : 0, 1); }
  void WriteUe(uint32_t v) {
    uint32_t code = v + 1;
    uint32_t nbits = 0;
    while ((code >> nbits) != 0) {
      ++nbits;
    }
    WriteBits(0, nbits - 1);
    WriteBits(code, nbits);
  }
  const std::vector<uint8_t>& bytes() const { return bytes_; }

 private:
  std::vector<uint8_t> bytes_;
  uint32_t bit_ = 0;
};

int main() {
  // Active parameter sets for the 1280x720 clip.
  auto sps_nals = WrapNal({0x42, 0x01, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00,
                           0x9e, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x5d, 0x90,
                           0x00, 0x50, 0x10, 0x05, 0xa2, 0xcb, 0x2b, 0x34, 0x92,
                           0x65, 0x78, 0x0b, 0x70, 0x20, 0x20, 0x00, 0x40, 0x00,
                           0x00, 0x03, 0x00, 0x40, 0x00, 0x00, 0x06, 0x42});
  Sps sps;
  CHECK(sps_nals.size() == 1 &&
        ParseSps(sps_nals[0].rbsp.data(), sps_nals[0].rbsp.size(), &sps));

  auto pps_nals = WrapNal({0x44, 0x01, 0xc1, 0x72, 0x86, 0x0c, 0x46, 0x24});
  Pps pps;
  CHECK(pps_nals.size() == 1 &&
        ParsePps(pps_nals[0].rbsp.data(), pps_nals[0].rbsp.size(), &pps));

  SliceContext ctx;
  ctx.pic_size_in_ctbs = sps.pic_size_in_ctbs;
  ctx.log2_max_pic_order_cnt_lsb = sps.log2_max_pic_order_cnt_lsb;
  ctx.separate_colour_plane_flag = sps.separate_colour_plane_flag;
  ctx.chroma_array_type =
      sps.separate_colour_plane_flag ? 0 : sps.chroma_format_idc;
  ctx.sample_adaptive_offset_enabled_flag =
      sps.sample_adaptive_offset_enabled_flag;
  ctx.sps_temporal_mvp_enabled_flag = sps.sps_temporal_mvp_enabled_flag;
  ctx.long_term_ref_pics_present_flag = sps.long_term_ref_pics_present_flag;
  ctx.num_long_term_ref_pics_sps = sps.num_long_term_ref_pics_sps;
  ctx.short_term_rps = sps.short_term_rps;
  ctx.dependent_slice_segments_enabled_flag =
      pps.dependent_slice_segments_enabled_flag;
  ctx.num_extra_slice_header_bits = pps.num_extra_slice_header_bits;
  ctx.output_flag_present_flag = pps.output_flag_present_flag;
  ctx.num_ref_idx_l0_default_active_minus1 =
      pps.num_ref_idx_l0_default_active_minus1;
  ctx.num_ref_idx_l1_default_active_minus1 =
      pps.num_ref_idx_l1_default_active_minus1;
  ctx.cabac_init_present_flag = pps.cabac_init_present_flag;
  ctx.weighted_pred_flag = pps.weighted_pred_flag;
  ctx.weighted_bipred_flag = pps.weighted_bipred_flag;
  ctx.pps_slice_chroma_qp_offsets_present_flag =
      pps.pps_slice_chroma_qp_offsets_present_flag;
  ctx.deblocking_filter_override_enabled_flag =
      pps.deblocking_filter_override_enabled_flag;
  ctx.pps_deblocking_filter_disabled_flag =
      pps.pps_deblocking_filter_disabled_flag;
  ctx.pps_loop_filter_across_slices_enabled_flag =
      pps.pps_loop_filter_across_slices_enabled_flag;
  ctx.tiles_enabled_flag = pps.tiles_enabled_flag;
  ctx.entropy_coding_sync_enabled_flag = pps.entropy_coding_sync_enabled_flag;
  ctx.lists_modification_present_flag = pps.lists_modification_present_flag;
  ctx.slice_segment_header_extension_present_flag =
      pps.slice_segment_header_extension_present_flag;

  // The clip is 4:4:4, so SAO chroma is present alongside SAO luma.
  CHECK(ctx.chroma_array_type != 0);
  // WPP is on, so the header carries entry-point offsets (one per CTB row past
  // the first): 720 / 64 = 12 CTB rows -> 11 entry points.
  CHECK(ctx.entropy_coding_sync_enabled_flag);

  // IDR (IDR_N_LP, type 20): an IRAP, so no_output_of_prior_pics is read; the
  // POC and RPS syntax is skipped for an IDR. Slice type I.
  {
    auto n = WrapNal({0x28, 0x01, 0xaf, 0x1d, 0x18, 0x2a, 0x67, 0xaf,
                      0x2d, 0xda, 0x1d, 0x36, 0x14, 0x8b, 0x89, 0xcf,
                      0xb2, 0xa6, 0x28, 0x68, 0xf7, 0xcf, 0x80, 0xff,
                      0xfb, 0x68, 0xc7, 0xfc, 0x53, 0x87, 0xf5, 0xa1});
    CHECK(n.size() == 1);
    SliceHeader sh;
    const bool ok = ParseSliceHeader(n[0].rbsp.data(), n[0].rbsp.size(),
                                     n[0].type, ctx, &sh);
    CHECK(ok);
    CHECK(sh.first_slice_segment_in_pic_flag);
    CHECK(sh.slice_type == SliceType::kI);
    CHECK(sh.slice_pic_order_cnt_lsb == 0);  // skipped for IDR
    CHECK(sh.current_rps.num_delta_pocs == 0);
    CHECK(sh.num_pic_total_curr == 0);
    CHECK(sh.slice_temporal_mvp_enabled_flag == false);  // skipped for IDR
    CHECK(sh.slice_sao_luma_flag);
    CHECK(sh.slice_sao_chroma_flag);
    CHECK(sh.num_entry_point_offsets == 11);  // 720 / 64 = 12 CTB rows
    CHECK(sh.slice_qp_delta == 7);
    CHECK(sh.slice_data_bit_offset_rbsp == 144);
    CHECK((sh.slice_data_bit_offset_rbsp & 7) == 0);  // byte-aligned
  }

  // P slice (TRAIL_R, type 1): POC 2, inline explicit RPS with one negative
  // reference, temporal MVP on.
  {
    auto n = WrapNal({0x02, 0x01, 0xd0, 0x11, 0x57, 0x84, 0x31, 0x8e,
                      0x0c, 0x38, 0x61, 0x01, 0x42, 0x05, 0x0c, 0x18,
                      0x2e, 0xfb, 0xc2, 0xe0, 0xf9, 0x8a, 0x6f, 0x20,
                      0x19, 0x98, 0x78, 0xf3, 0x92, 0x31, 0x17, 0x18});
    CHECK(n.size() == 1);
    SliceHeader sh;
    const bool ok = ParseSliceHeader(n[0].rbsp.data(), n[0].rbsp.size(),
                                     n[0].type, ctx, &sh);
    CHECK(ok);
    CHECK(sh.first_slice_segment_in_pic_flag);
    CHECK(sh.slice_type == SliceType::kP);
    CHECK(sh.slice_pic_order_cnt_lsb == 2);
    CHECK(sh.short_term_ref_pic_set_sps_flag == false);  // inline
    CHECK(sh.short_term_ref_pic_set_bits == 8);
    CHECK(sh.current_rps.num_negative_pics == 1);
    CHECK(sh.current_rps.num_positive_pics == 0);
    CHECK(sh.num_pic_total_curr == 1);
    CHECK(sh.slice_temporal_mvp_enabled_flag);
    CHECK(sh.slice_sao_luma_flag);
    // The clip's PPS enables weighted prediction, so a P slice carries a
    // pred_weight_table; libx265 signals the denominators but leaves every
    // luma_weight_flag off (default weights).
    CHECK(sh.pred_weight.luma_log2_weight_denom == 7);
    CHECK(sh.pred_weight.delta_chroma_log2_weight_denom == -1);
    CHECK(sh.pred_weight.luma_weight_flag[0][0] == false);
    CHECK(sh.slice_qp_delta == 7);
    CHECK(sh.slice_data_bit_offset_rbsp == 144);
    CHECK((sh.slice_data_bit_offset_rbsp & 7) == 0);
  }

  // B slice (TRAIL_N, type 0): POC 1, inline RPS with a reference on each side.
  {
    auto n = WrapNal({0x00, 0x01, 0xe0, 0x24, 0xbf, 0x86, 0x14, 0x8c,
                      0x22, 0xa9, 0x99, 0xa2, 0x15, 0xea, 0xc0, 0x8b,
                      0x40, 0x01, 0x28, 0x9d, 0x80, 0x94, 0x37, 0xc8,
                      0x63, 0x7f, 0xe4, 0x65, 0xa3, 0xa4, 0x14, 0x69});
    CHECK(n.size() == 1);
    SliceHeader sh;
    const bool ok = ParseSliceHeader(n[0].rbsp.data(), n[0].rbsp.size(),
                                     n[0].type, ctx, &sh);
    CHECK(ok);
    CHECK(sh.slice_type == SliceType::kB);
    CHECK(sh.slice_pic_order_cnt_lsb == 1);
    CHECK(sh.current_rps.num_negative_pics == 1);
    CHECK(sh.current_rps.num_positive_pics == 1);
    CHECK(sh.num_pic_total_curr == 2);
    CHECK(sh.slice_temporal_mvp_enabled_flag);
    CHECK(sh.slice_qp_delta == 10);
    CHECK(sh.slice_data_bit_offset_rbsp == 104);
    CHECK((sh.slice_data_bit_offset_rbsp & 7) == 0);
  }

  // Truncation and null inputs are rejected, not over-read.
  {
    SliceHeader sh;
    CHECK(!ParseSliceHeader(nullptr, 4, NalUnitType::kTrailR, ctx, &sh));
    const uint8_t dummy = 0;
    CHECK(!ParseSliceHeader(&dummy, 1, NalUnitType::kTrailN, ctx, nullptr));
  }

  // Synthetic P slice exercising ref_pic_lists_modification: an SPS-defined RPS
  // with two used references and an L0 reorder that swaps them.
  {
    SliceContext mctx;
    mctx.pic_size_in_ctbs = 1;  // first slice, so no segment address
    mctx.log2_max_pic_order_cnt_lsb = 4;
    mctx.chroma_array_type = 1;
    mctx.lists_modification_present_flag = true;
    ShortTermRps rps;
    rps.num_negative_pics = 2;
    rps.num_delta_pocs = 2;
    rps.delta_poc_s0[0] = -1;
    rps.delta_poc_s0[1] = -2;
    rps.used_s0[0] = true;
    rps.used_s0[1] = true;
    mctx.short_term_rps = {rps};

    BitWriter w;
    w.WriteFlag(true);  // first_slice_segment_in_pic_flag
    w.WriteUe(0);       // slice_pic_parameter_set_id
    w.WriteUe(1);       // slice_type = P
    w.WriteBits(5, 4);  // slice_pic_order_cnt_lsb
    w.WriteFlag(true);  // short_term_ref_pic_set_sps_flag (num_st_rps == 1)
    // (no SAO, no temporal MVP.) P-slice reference block:
    w.WriteFlag(true);  // num_ref_idx_active_override_flag
    w.WriteUe(1);       // num_ref_idx_l0_active_minus1 = 1 (two active)
    // ref_pic_lists_modification: NumPicTotalCurr = 2, so entry width is 1 bit.
    w.WriteFlag(true);  // ref_pic_list_modification_flag_l0
    w.WriteBits(1, 1);  // list_entry_l0[0] = 1
    w.WriteBits(0, 1);  // list_entry_l0[1] = 0
    w.WriteUe(0);       // five_minus_max_num_merge_cand
    w.WriteUe(0);       // slice_qp_delta se(0) == ue(0)
    w.WriteFlag(true);  // byte_alignment: alignment_bit_equal_to_one
    const std::vector<uint8_t> rbsp = w.bytes();

    SliceHeader sh;
    const bool ok = ParseSliceHeader(rbsp.data(), rbsp.size(),
                                     NalUnitType::kTrailR, mctx, &sh);
    CHECK(ok);
    CHECK(sh.slice_type == SliceType::kP);
    CHECK(sh.num_pic_total_curr == 2);
    CHECK(sh.num_ref_idx_l0_active_minus1 == 1);
    CHECK(sh.ref_pic_list_modification_flag_l0 == true);
    CHECK(sh.list_entry_l0[0] == 1);
    CHECK(sh.list_entry_l0[1] == 0);
  }

  // A list_entry that indexes outside the temporary list (>= NumPicTotalCurr)
  // is rejected. With three references the entry is 2 bits wide, so the value 3
  // is representable but out of range.
  {
    SliceContext mctx;
    mctx.pic_size_in_ctbs = 1;
    mctx.log2_max_pic_order_cnt_lsb = 4;
    mctx.chroma_array_type = 1;
    mctx.lists_modification_present_flag = true;
    ShortTermRps rps;
    rps.num_negative_pics = 3;
    rps.num_delta_pocs = 3;
    rps.delta_poc_s0[0] = -1;
    rps.delta_poc_s0[1] = -2;
    rps.delta_poc_s0[2] = -3;
    rps.used_s0[0] = true;
    rps.used_s0[1] = true;
    rps.used_s0[2] = true;
    mctx.short_term_rps = {rps};

    BitWriter w;
    w.WriteFlag(true);  // first_slice_segment_in_pic_flag
    w.WriteUe(0);       // slice_pic_parameter_set_id
    w.WriteUe(1);       // slice_type = P
    w.WriteBits(5, 4);  // slice_pic_order_cnt_lsb
    w.WriteFlag(true);  // short_term_ref_pic_set_sps_flag
    w.WriteFlag(true);  // num_ref_idx_active_override_flag
    w.WriteUe(0);       // num_ref_idx_l0_active_minus1 = 0 (one active)
    w.WriteFlag(true);  // ref_pic_list_modification_flag_l0
    w.WriteBits(3, 2);  // list_entry_l0[0] = 3 >= NumPicTotalCurr 3 -> reject
    const std::vector<uint8_t> rbsp = w.bytes();

    SliceHeader sh;
    CHECK(!ParseSliceHeader(rbsp.data(), rbsp.size(), NalUnitType::kTrailR,
                            mctx, &sh));
  }

  // Synthetic P slice with long-term references: one from the SPS (by index)
  // and one signalled explicitly with a delta-POC-MSB cycle.
  {
    SliceContext lctx;
    lctx.pic_size_in_ctbs = 1;
    lctx.log2_max_pic_order_cnt_lsb = 4;
    lctx.chroma_array_type = 1;
    lctx.long_term_ref_pics_present_flag = true;
    lctx.num_long_term_ref_pics_sps = 1;
    lctx.used_by_curr_pic_lt_sps = {true};
    lctx.lt_ref_pic_poc_lsb_sps = {3};
    ShortTermRps empty_rps;  // no short-term references
    lctx.short_term_rps = {empty_rps};

    BitWriter w;
    w.WriteFlag(true);  // first_slice_segment_in_pic_flag
    w.WriteUe(0);       // slice_pic_parameter_set_id
    w.WriteUe(1);       // slice_type = P
    w.WriteBits(5, 4);  // slice_pic_order_cnt_lsb
    w.WriteFlag(true);  // short_term_ref_pic_set_sps_flag (num_st_rps == 1)
    // long-term references:
    w.WriteUe(1);  // num_long_term_sps
    w.WriteUe(1);  // num_long_term_pics
    // entry 0 (from SPS, only one entry so no lt_idx_sps):
    w.WriteFlag(false);  // delta_poc_msb_present_flag[0]
    // entry 1 (explicit):
    w.WriteBits(7, 4);   // poc_lsb_lt[1]
    w.WriteFlag(false);  // used_by_curr_pic_lt_flag[1]
    w.WriteFlag(true);   // delta_poc_msb_present_flag[1]
    w.WriteUe(2);        // delta_poc_msb_cycle_lt[1]
    // P reference block: no override (defaults), NumPicTotalCurr = 1.
    w.WriteFlag(false);  // num_ref_idx_active_override_flag
    w.WriteUe(0);        // five_minus_max_num_merge_cand
    w.WriteUe(0);        // slice_qp_delta se(0)
    w.WriteFlag(true);   // byte_alignment
    const std::vector<uint8_t> rbsp = w.bytes();

    SliceHeader sh;
    const bool ok = ParseSliceHeader(rbsp.data(), rbsp.size(),
                                     NalUnitType::kTrailR, lctx, &sh);
    CHECK(ok);
    CHECK(sh.num_long_term_sps == 1);
    CHECK(sh.num_long_term_pics == 1);
    CHECK(sh.long_term_refs[0].poc_lsb == 3);
    CHECK(sh.long_term_refs[0].used_by_curr == true);
    CHECK(sh.long_term_refs[0].delta_poc_msb_present == false);
    CHECK(sh.long_term_refs[1].poc_lsb == 7);
    CHECK(sh.long_term_refs[1].used_by_curr == false);
    CHECK(sh.long_term_refs[1].delta_poc_msb_present == true);
    CHECK(sh.long_term_refs[1].delta_poc_msb_cycle == 2);
    // Only the used long-term reference (entry 0) counts toward
    // NumPicTotalCurr.
    CHECK(sh.num_pic_total_curr == 1);
  }

  // Synthetic IDR I slice with the PPS chroma-QP-offset list enabled: the slice
  // signals cu_chroma_qp_offset_enabled_flag (range extension).
  {
    SliceContext cctx;
    cctx.pic_size_in_ctbs = 1;
    cctx.chroma_array_type = 1;
    cctx.pps_slice_chroma_qp_offsets_present_flag = true;
    cctx.chroma_qp_offset_list_enabled_flag = true;

    BitWriter w;
    w.WriteFlag(true);   // first_slice_segment_in_pic_flag
    w.WriteFlag(false);  // no_output_of_prior_pics_flag (IDR is an IRAP)
    w.WriteUe(0);        // slice_pic_parameter_set_id
    w.WriteUe(2);        // slice_type = I
    // (IDR: no POC / RPS; SAO off; no P/B block.)
    w.WriteUe(0);       // slice_qp_delta se(0)
    w.WriteUe(0);       // slice_cb_qp_offset se(0)
    w.WriteUe(0);       // slice_cr_qp_offset se(0)
    w.WriteFlag(true);  // cu_chroma_qp_offset_enabled_flag
    w.WriteFlag(true);  // byte_alignment
    const std::vector<uint8_t> rbsp = w.bytes();

    SliceHeader sh;
    const bool ok = ParseSliceHeader(rbsp.data(), rbsp.size(),
                                     NalUnitType::kIdrWRadl, cctx, &sh);
    CHECK(ok);
    CHECK(sh.slice_type == SliceType::kI);
    CHECK(sh.cu_chroma_qp_offset_enabled_flag == true);
  }

  // Synthetic dependent slice segment: parsing ends after the segment address.
  {
    SliceContext dctx;
    dctx.pic_size_in_ctbs = 240;  // CeilLog2 = 8 bits for the address
    dctx.dependent_slice_segments_enabled_flag = true;
    BitWriter w;
    w.WriteFlag(false);  // first_slice_segment_in_pic_flag
    w.WriteUe(0);        // slice_pic_parameter_set_id
    w.WriteFlag(true);   // dependent_slice_segment_flag
    w.WriteBits(5, 8);   // slice_segment_address
    const std::vector<uint8_t> rbsp = w.bytes();
    SliceHeader sh;
    const bool ok = ParseSliceHeader(rbsp.data(), rbsp.size(),
                                     NalUnitType::kTrailR, dctx, &sh);
    CHECK(ok);
    CHECK(sh.first_slice_segment_in_pic_flag == false);
    CHECK(sh.dependent_slice_segment_flag == true);
    CHECK(sh.slice_segment_address == 5);
  }

  // A segment address outside the picture is rejected. With 3 CTBs the address
  // is a 2-bit field (CeilLog2(3) = 2), so the value 3 is representable but out
  // of range.
  {
    SliceContext dctx;
    dctx.pic_size_in_ctbs = 3;
    BitWriter w;
    w.WriteFlag(false);  // first_slice_segment_in_pic_flag
    w.WriteUe(0);        // slice_pic_parameter_set_id
    w.WriteBits(3, 2);   // slice_segment_address = 3 >= 3 -> rejected
    SliceHeader sh;
    CHECK(!ParseSliceHeader(w.bytes().data(), w.bytes().size(),
                            NalUnitType::kTrailR, dctx, &sh));
  }

  // Weighted-prediction offset derivation (clause 7.4.7.3).
  {
    // WpOffsetHalfRange: 8-bit fixed (1 << 7) without high precision, at any
    // coded bit depth; full bit-depth range (1 << (BitDepth-1)) with it.
    CHECK(WpOffsetHalfRange(8, false) == 128);
    CHECK(WpOffsetHalfRange(10, false) == 128);  // NOT 512 without high prec.
    CHECK(WpOffsetHalfRange(12, false) == 128);
    CHECK(WpOffsetHalfRange(8, true) == 128);
    CHECK(WpOffsetHalfRange(10, true) == 512);
    CHECK(WpOffsetHalfRange(12, true) == 2048);

    // DeriveChromaOffset: with a default chroma weight (== 1 << denom) the
    // weighting term cancels the half range, so the result is exactly
    // delta_chroma_offset, clipped to [-half, half-1].
    const int denom = 6;
    const int default_w = 1 << denom;
    CHECK(DeriveChromaOffset(128, 0, default_w, denom) == 0);
    CHECK(DeriveChromaOffset(128, 40, default_w, denom) == 40);
    CHECK(DeriveChromaOffset(128, -40, default_w, denom) == -40);
    // Clipping to the half-range bounds.
    CHECK(DeriveChromaOffset(128, 1000, default_w, denom) == 127);
    CHECK(DeriveChromaOffset(128, -1000, default_w, denom) == -128);
    // High precision at 10-bit: a value that would overflow int8 survives,
    // proving the wide path is needed (it must reach the int16 VA field).
    CHECK(DeriveChromaOffset(512, 300, default_w, denom) == 300);
    CHECK(DeriveChromaOffset(512, -300, default_w, denom) == -300);
    CHECK(DeriveChromaOffset(512, 10000, default_w, denom) == 511);
    // A non-default weight shifts the derived offset by the weighting term:
    // half + delta - ((half * w) >> denom). half=128, w=128 (2x), denom=6 ->
    // 128 + 10 - ((128*128)>>6) = 128 + 10 - 256 = -118.
    CHECK(DeriveChromaOffset(128, 10, 128, denom) == -118);
  }

  if (g_failures == 0) {
    std::printf("H265_SLICE_HEADER_TEST_OK\n");
    return 0;
  }
  std::printf("H265_SLICE_HEADER_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.265 / HEVC slice-segment header parsing, through the reference-picture
// information: slice identification (first-slice / segment address / type /
// dependent flag), the picture order count, the short-term and long-term
// reference-picture-set selection, the SAO flags, and the active reference
// counts. These are the fields the demuxer and the picture-output scheduler
// need; a full stateless-decode path would continue past here into
// ref_pic_lists_modification, pred_weight_table, the deblocking overrides, and
// the entry-point offsets, which are out of scope for this parser.
//
// The header is variable-length and attacker-controlled, so every field is read
// through the bounds-checked BitReader and every count that bounds a loop is
// range-limited. Parsing needs the active SPS/PPS state, supplied via
// SliceContext.
#ifndef V4L2WC_PARSE_H265_SLICE_HEADER_H_
#define V4L2WC_PARSE_H265_SLICE_HEADER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "parse/bit_reader.h"
#include "parse/h265/nal.h"
#include "parse/h265/pps.h"
#include "parse/h265/sps.h"

namespace v4l2wc::h265 {

// HEVC slice types (clause 7.4.7.1).
enum class SliceType : uint32_t { kB = 0, kP = 1, kI = 2 };

// Largest number of active references per list (num_ref_idx_lX_active_minus1 is
// at most 14, so at most 15 entries, indices 0..14).
inline constexpr uint32_t kMaxSliceRefs = 15;

// Upper bound on num_long_term_sps + num_long_term_pics; each term is
// DPB-bounded (clause 7.4.7.1).
inline constexpr uint32_t kMaxLongTermTotal = kMaxLongTermRps + kMaxRefPics;

// A long-term reference the current slice signals (clause 7.3.6.1), resolved
// against the SPS for the by-index entries. A stateless decoder derives its POC
// from these and PicOrderCntVal.
struct LongTermRef {
  uint32_t poc_lsb = 0;       // PocLsbLt
  bool used_by_curr = false;  // UsedByCurrPicLt
  bool delta_poc_msb_present = false;
  int32_t delta_poc_msb_cycle = 0;  // cumulative DeltaPocMsbCycleLt
};

// pred_weight_table (clause 7.3.6.3), as raw syntax elements per list (0 = L0,
// 1 = L1) and reference index. Present only for a weighted P or B slice; a
// stateless decoder derives the final weights and offsets from these.
struct PredWeightTable {
  uint32_t luma_log2_weight_denom = 0;
  int32_t delta_chroma_log2_weight_denom = 0;
  bool luma_weight_flag[2][kMaxSliceRefs] = {};
  int32_t delta_luma_weight[2][kMaxSliceRefs] = {};
  int32_t luma_offset[2][kMaxSliceRefs] = {};
  bool chroma_weight_flag[2][kMaxSliceRefs] = {};
  int32_t delta_chroma_weight[2][kMaxSliceRefs][2] = {};
  int32_t delta_chroma_offset[2][kMaxSliceRefs][2] = {};
};

// Fields the slice-segment header parse needs from the active SPS and PPS.
struct SliceContext {
  SliceContext();

  // From SPS.
  uint32_t pic_size_in_ctbs = 0;  // sizes slice_segment_address's u(v)
  uint32_t log2_max_pic_order_cnt_lsb = 4;
  bool separate_colour_plane_flag = false;
  // ChromaArrayType: chroma_format_idc, or 0 when monochrome or the colour
  // planes are coded separately. Gates slice_sao_chroma_flag.
  uint32_t chroma_array_type = 1;
  bool sample_adaptive_offset_enabled_flag = false;
  bool sps_temporal_mvp_enabled_flag = false;
  bool long_term_ref_pics_present_flag = false;
  uint32_t num_long_term_ref_pics_sps = 0;
  // Per SPS long-term entry (from the SPS); size() is
  // num_long_term_ref_pics_sps. A by-index long-term reference resolves its POC
  // LSB and used flag through these.
  std::vector<bool> used_by_curr_pic_lt_sps;
  std::vector<uint32_t> lt_ref_pic_poc_lsb_sps;
  // The SPS-defined short-term RPSs. size() is num_short_term_ref_pic_sets.
  std::vector<ShortTermRps> short_term_rps;

  // From PPS.
  bool dependent_slice_segments_enabled_flag = false;
  uint32_t num_extra_slice_header_bits = 0;
  bool output_flag_present_flag = false;
  uint32_t num_ref_idx_l0_default_active_minus1 = 0;
  uint32_t num_ref_idx_l1_default_active_minus1 = 0;
  bool cabac_init_present_flag = false;
  bool weighted_pred_flag = false;
  bool weighted_bipred_flag = false;
  bool pps_slice_chroma_qp_offsets_present_flag = false;
  bool deblocking_filter_override_enabled_flag = false;
  bool pps_deblocking_filter_disabled_flag = false;
  bool pps_loop_filter_across_slices_enabled_flag = false;
  bool tiles_enabled_flag = false;
  bool entropy_coding_sync_enabled_flag = false;
  bool lists_modification_present_flag = false;
  bool slice_segment_header_extension_present_flag = false;
};

struct SliceHeader {
  SliceHeader();
  ~SliceHeader();
  SliceHeader(const SliceHeader&);
  SliceHeader& operator=(const SliceHeader&);
  SliceHeader(SliceHeader&&) noexcept;
  SliceHeader& operator=(SliceHeader&&) noexcept;

  bool first_slice_segment_in_pic_flag = false;
  bool no_output_of_prior_pics_flag = false;  // IRAP only
  uint32_t slice_pic_parameter_set_id = 0;
  bool dependent_slice_segment_flag = false;
  uint32_t slice_segment_address = 0;

  SliceType slice_type = SliceType::kI;
  bool pic_output_flag = true;
  uint32_t colour_plane_id = 0;

  // Present for non-IDR slices.
  uint32_t slice_pic_order_cnt_lsb = 0;
  bool short_term_ref_pic_set_sps_flag = false;
  uint32_t short_term_ref_pic_set_idx = 0;
  // Bit size of the inline st_ref_pic_set(num_short_term_ref_pic_sets) when
  // short_term_ref_pic_set_sps_flag is 0; 0 otherwise. A hardware decoder is
  // told this so it can skip re-parsing that structure (VAAPI st_rps_bits).
  uint32_t short_term_ref_pic_set_bits = 0;
  // The short-term RPS in effect for this slice: selected from the SPS by
  // index, or derived inline. Empty for IDR slices.
  ShortTermRps current_rps;
  uint32_t num_long_term_sps = 0;
  uint32_t num_long_term_pics = 0;
  // The long-term references, indices 0..num_long_term_sps+num_long_term_pics-1
  // (present only when the SPS enables long-term references).
  LongTermRef long_term_refs[kMaxLongTermTotal] = {};
  bool slice_temporal_mvp_enabled_flag = false;

  bool slice_sao_luma_flag = false;
  bool slice_sao_chroma_flag = false;

  bool num_ref_idx_active_override_flag = false;
  // Resolved active counts: the override value if present, else the PPS
  // default.
  uint32_t num_ref_idx_l0_active_minus1 = 0;
  uint32_t num_ref_idx_l1_active_minus1 = 0;
  // Populated only for a weighted P or B slice (see ctx.weighted_pred_flag /
  // weighted_bipred_flag); otherwise left default.
  PredWeightTable pred_weight;

  // ref_pic_lists_modification (clause 7.3.6.2). The flags are set only when
  // lists_modification_present_flag is set and NumPicTotalCurr > 1; list_entry
  // is valid for indices 0..num_ref_idx_lX_active_minus1 and each entry is an
  // index into the temporary reference list (< NumPicTotalCurr).
  bool ref_pic_list_modification_flag_l0 = false;
  bool ref_pic_list_modification_flag_l1 = false;
  uint32_t list_entry_l0[kMaxSliceRefs] = {};
  uint32_t list_entry_l1[kMaxSliceRefs] = {};

  // NumPicTotalCurr (clause 7.4.7.2): the number of reference pictures used by
  // the current picture, which gates ref_pic_lists_modification and sizes its
  // list_entry fields.
  uint32_t num_pic_total_curr = 0;
  bool mvd_l1_zero_flag = false;
  bool cabac_init_flag = false;
  bool collocated_from_l0_flag = true;
  uint32_t collocated_ref_idx = 0;
  uint32_t five_minus_max_num_merge_cand = 0;

  int32_t slice_qp_delta = 0;
  int32_t slice_cb_qp_offset = 0;
  int32_t slice_cr_qp_offset = 0;
  bool slice_deblocking_filter_disabled_flag = false;
  int32_t slice_beta_offset_div2 = 0;
  int32_t slice_tc_offset_div2 = 0;
  bool slice_loop_filter_across_slices_enabled_flag = false;
  uint32_t num_entry_point_offsets = 0;

  // Bit offset within the RBSP at which slice_data() begins, i.e. just past the
  // header's byte_alignment(). Always a multiple of 8. Convert to raw-NAL bit
  // space with RbspToRawBitOffset() for a stateless decoder's
  // slice_data_bit_offset.
  uint32_t slice_data_bit_offset_rbsp = 0;
};

// Parses a slice-segment header from its RBSP (NAL header stripped,
// emulation-prevention bytes removed) through the active reference counts.
// `nal_type` comes from the NAL header and selects the IRAP / IDR branches.
// Returns true and fills *out on success, false on malformed or out-of-range
// input.
bool ParseSliceHeader(const uint8_t* rbsp, size_t size, NalUnitType nal_type,
                      const SliceContext& ctx, SliceHeader* out);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_SLICE_HEADER_H_

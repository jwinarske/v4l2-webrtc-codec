// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.265 / HEVC Picture Parameter Set parsing. Recovers the fields a stateless
// decoder needs for its picture parameters and that the slice-segment header
// parser consults (extra slice-header bits, default reference counts, tiles /
// wavefront layout, deblocking and QP defaults, and the reference-list and
// extension presence flags).
//
// Attacker-controlled input: every field is read through the bounds-checked
// BitReader and each count that bounds a loop is range-limited, so malformed
// input fails the parse rather than over-reading or looping unboundedly.
#ifndef V4L2WC_PARSE_H265_PPS_H_
#define V4L2WC_PARSE_H265_PPS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "parse/h265/scaling_list.h"

namespace v4l2wc::h265 {

// Bound on num_tile_columns_minus1 / num_tile_rows_minus1. The spec ties these
// to PicWidthInCtbsY / PicHeightInCtbsY, each at most kMaxDimension / 16 =
// 1024; the PPS is parsed without SPS context, so this bound stands in for that
// and caps the column-width / row-height loops.
inline constexpr uint32_t kMaxTiles = 1024;

// Spec maximum for num_ref_idx_lX_default_active_minus1 (clause 7.4.3.3.1).
inline constexpr uint32_t kMaxRefIdxDefault = 14;

struct Pps {
  Pps();

  uint32_t pps_pic_parameter_set_id = 0;
  uint32_t pps_seq_parameter_set_id = 0;  // the SPS this PPS references

  bool dependent_slice_segments_enabled_flag = false;
  bool output_flag_present_flag = false;
  uint32_t num_extra_slice_header_bits = 0;
  bool sign_data_hiding_enabled_flag = false;
  bool cabac_init_present_flag = false;

  uint32_t num_ref_idx_l0_default_active_minus1 = 0;
  uint32_t num_ref_idx_l1_default_active_minus1 = 0;
  int32_t init_qp_minus26 = 0;
  bool constrained_intra_pred_flag = false;
  bool transform_skip_enabled_flag = false;
  bool cu_qp_delta_enabled_flag = false;
  uint32_t diff_cu_qp_delta_depth = 0;
  int32_t pps_cb_qp_offset = 0;
  int32_t pps_cr_qp_offset = 0;
  bool pps_slice_chroma_qp_offsets_present_flag = false;

  bool weighted_pred_flag = false;
  bool weighted_bipred_flag = false;
  bool transquant_bypass_enabled_flag = false;
  bool tiles_enabled_flag = false;
  bool entropy_coding_sync_enabled_flag = false;

  uint32_t num_tile_columns_minus1 = 0;
  uint32_t num_tile_rows_minus1 = 0;
  bool uniform_spacing_flag = true;
  // Explicit per-tile widths/heights in CTBs, present only when
  // uniform_spacing_flag is false. Each holds num_tile_columns_minus1 /
  // num_tile_rows_minus1 entries (the last column/row is derived, not coded).
  std::vector<uint32_t> column_width_minus1;
  std::vector<uint32_t> row_height_minus1;
  bool loop_filter_across_tiles_enabled_flag = true;

  bool pps_loop_filter_across_slices_enabled_flag = false;
  bool deblocking_filter_control_present_flag = false;
  bool deblocking_filter_override_enabled_flag = false;
  bool pps_deblocking_filter_disabled_flag = false;
  int32_t pps_beta_offset_div2 = 0;
  int32_t pps_tc_offset_div2 = 0;

  bool pps_scaling_list_data_present_flag = false;
  // Scaling lists signalled by the PPS (present only when the flag above is
  // set); these override the SPS lists for the picture.
  ScalingListData scaling_list;
  bool lists_modification_present_flag = false;
  uint32_t log2_parallel_merge_level_minus2 = 0;
  bool slice_segment_header_extension_present_flag = false;
};

// Parses a PPS from its RBSP (NAL header stripped, emulation-prevention bytes
// removed). Returns true and fills *out on success, false on malformed input.
bool ParsePps(const uint8_t* rbsp, size_t size, Pps* out);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_PPS_H_

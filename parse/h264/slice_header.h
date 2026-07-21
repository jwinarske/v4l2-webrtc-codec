// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.264 slice-header parsing, through the reference-picture syntax
// (ref_pic_list_modification and dec_ref_pic_marking / MMCO). These
// variable-length, attacker-controlled structures are what a decoder turns
// into V4L2 decode parameters and reference lists, so they are the primary
// memory-safety surface: every field is bounds-checked and every list is
// count-limited. Parsing needs the active SPS/PPS state, supplied via
// SliceContext.
#ifndef V4L2WC_PARSE_H264_SLICE_HEADER_H_
#define V4L2WC_PARSE_H264_SLICE_HEADER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace v4l2wc::h264 {

// Fields the slice header parse needs from the active SPS and PPS.
struct SliceContext {
  // From SPS.
  uint32_t log2_max_frame_num = 4;  // log2_max_frame_num_minus4 + 4
  uint32_t pic_order_cnt_type = 0;
  uint32_t log2_max_pic_order_cnt_lsb = 4;        // poc_type 0
  bool delta_pic_order_always_zero_flag = false;  // poc_type 1
  bool frame_mbs_only_flag = true;
  bool separate_colour_plane_flag = false;
  uint32_t chroma_array_type = 1;  // 0 for monochrome / separate plane
  // From PPS.
  bool bottom_field_pic_order_in_frame_present_flag = false;
  uint32_t num_ref_idx_l0_default_active_minus1 = 0;
  uint32_t num_ref_idx_l1_default_active_minus1 = 0;
  bool weighted_pred_flag = false;
  uint32_t weighted_bipred_idc = 0;
  bool redundant_pic_cnt_present_flag = false;
};

struct RefPicListMod {
  uint32_t idc = 0;    // modification_of_pic_nums_idc (0..3; 3 terminates)
  uint32_t value = 0;  // abs_diff_pic_num_minus1 or long_term_pic_num
};

struct Mmco {
  uint32_t op = 0;  // memory_management_control_operation (0 terminates)
  uint32_t arg1 = 0;
  uint32_t arg2 = 0;
};

struct SliceHeader {
  uint32_t first_mb_in_slice = 0;
  uint32_t slice_type = 0;  // 0..9; % 5 gives P/B/I/SP/SI
  uint32_t pic_parameter_set_id = 0;
  uint32_t frame_num = 0;
  bool field_pic_flag = false;
  bool bottom_field_flag = false;
  bool idr = false;
  uint32_t idr_pic_id = 0;
  uint32_t pic_order_cnt_lsb = 0;
  uint32_t num_ref_idx_l0_active_minus1 = 0;
  uint32_t num_ref_idx_l1_active_minus1 = 0;
  std::vector<RefPicListMod> ref_pic_list_mod_l0;
  std::vector<RefPicListMod> ref_pic_list_mod_l1;
  // dec_ref_pic_marking.
  bool no_output_of_prior_pics_flag = false;        // IDR
  bool long_term_reference_flag = false;            // IDR
  bool adaptive_ref_pic_marking_mode_flag = false;  // non-IDR
  std::vector<Mmco> mmco;
};

// Parses a slice header from its RBSP (NAL header stripped,
// emulation-prevention bytes removed) up to and including dec_ref_pic_marking.
// `nal_ref_idc` and `idr` come from the NAL header (idr == (nal_unit_type ==
// 5)). Returns true and fills *out on success, false on malformed or
// out-of-range input.
bool ParseSliceHeader(const uint8_t* rbsp, size_t size, uint32_t nal_ref_idc,
                      bool idr, const SliceContext& ctx, SliceHeader* out);

}  // namespace v4l2wc::h264

#endif  // V4L2WC_PARSE_H264_SLICE_HEADER_H_

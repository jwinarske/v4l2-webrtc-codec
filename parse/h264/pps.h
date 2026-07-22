// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.264 Picture Parameter Set (PPS) parsing. Recovers the fields the decoder
// needs (entropy mode, default reference counts, initial QP, and the SPS it
// refers to). Attacker-controlled input: every field is read through the
// bounds-checked BitReader and the slice-group map is size-bounded.
#ifndef V4L2WC_PARSE_H264_PPS_H_
#define V4L2WC_PARSE_H264_PPS_H_

#include <cstddef>
#include <cstdint>

namespace v4l2wc::h264 {

struct Pps {
  Pps();

  uint32_t pic_parameter_set_id = 0;
  uint32_t seq_parameter_set_id = 0;      // the SPS this PPS references
  bool entropy_coding_mode_flag = false;  // true = CABAC, false = CAVLC
  uint32_t num_ref_idx_l0_default_active_minus1 = 0;
  uint32_t num_ref_idx_l1_default_active_minus1 = 0;
  int32_t pic_init_qp_minus26 = 0;
  bool deblocking_filter_control_present_flag = false;
  bool transform_8x8_mode_flag = false;

  // Additional fields a hardware decoder needs for its picture parameters and
  // to populate a SliceContext.
  uint32_t num_slice_groups_minus1 = 0;
  bool bottom_field_pic_order_in_frame_present_flag = false;
  bool weighted_pred_flag = false;
  uint32_t weighted_bipred_idc = 0;
  bool constrained_intra_pred_flag = false;
  bool redundant_pic_cnt_present_flag = false;
  int32_t chroma_qp_index_offset = 0;
  // Defaults to chroma_qp_index_offset when the optional extension is absent.
  int32_t second_chroma_qp_index_offset = 0;
};

// Parses a PPS from its RBSP (NAL header stripped, emulation-prevention bytes
// removed). Returns true and fills *out on success, false on malformed input.
bool ParsePps(const uint8_t* rbsp, size_t size, Pps* out);

}  // namespace v4l2wc::h264

#endif  // V4L2WC_PARSE_H264_PPS_H_

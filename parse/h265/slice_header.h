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
  // The SPS-defined short-term RPSs. size() is num_short_term_ref_pic_sets.
  std::vector<ShortTermRps> short_term_rps;

  // From PPS.
  bool dependent_slice_segments_enabled_flag = false;
  uint32_t num_extra_slice_header_bits = 0;
  bool output_flag_present_flag = false;
  uint32_t num_ref_idx_l0_default_active_minus1 = 0;
  uint32_t num_ref_idx_l1_default_active_minus1 = 0;
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
  // The short-term RPS in effect for this slice: selected from the SPS by
  // index, or derived inline. Empty for IDR slices.
  ShortTermRps current_rps;
  uint32_t num_long_term_sps = 0;
  uint32_t num_long_term_pics = 0;
  bool slice_temporal_mvp_enabled_flag = false;

  bool slice_sao_luma_flag = false;
  bool slice_sao_chroma_flag = false;

  bool num_ref_idx_active_override_flag = false;
  // Resolved active counts: the override value if present, else the PPS
  // default.
  uint32_t num_ref_idx_l0_active_minus1 = 0;
  uint32_t num_ref_idx_l1_active_minus1 = 0;
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

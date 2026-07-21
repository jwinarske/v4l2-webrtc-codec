// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h264/pps.h"

#include "parse/bit_reader.h"

namespace v4l2wc::h264 {
namespace {

// Spec caps: a picture has at most 8 slice groups, and a sane upper bound on
// the explicit slice-group map keeps the type-6 loop from being a DoS.
constexpr uint32_t kMaxSliceGroupsMinus1 = 7;
constexpr uint32_t kMaxPicSizeInMapUnits = 1u << 22;

// Consumes the slice-group map (spec 7.3.2.2) when num_slice_groups_minus1 > 0.
// All reads are bounds-checked; the counts are range-limited above.
bool SkipSliceGroupMap(BitReader* br, uint32_t num_slice_groups_minus1) {
  uint32_t map_type = 0;
  if (!br->ReadUe(&map_type)) {
    return false;
  }
  uint32_t ue = 0;
  switch (map_type) {
    case 0:
      for (uint32_t i = 0; i <= num_slice_groups_minus1; ++i) {
        if (!br->ReadUe(&ue)) {  // run_length_minus1[i]
          return false;
        }
      }
      return true;
    case 2:
      for (uint32_t i = 0; i < num_slice_groups_minus1; ++i) {
        if (!br->ReadUe(&ue) || !br->ReadUe(&ue)) {  // top_left / bottom_right
          return false;
        }
      }
      return true;
    case 3:
    case 4:
    case 5:
      return br->SkipBits(1) &&  // slice_group_change_direction_flag
             br->ReadUe(&ue);    // slice_group_change_rate_minus1
    case 6: {
      uint32_t pic_size_in_map_units_minus1 = 0;
      if (!br->ReadUe(&pic_size_in_map_units_minus1)) {
        return false;
      }
      if (pic_size_in_map_units_minus1 >= kMaxPicSizeInMapUnits) {
        return false;
      }
      uint32_t bits = 0;
      while ((1u << bits) < (num_slice_groups_minus1 + 1)) {
        ++bits;  // Ceil(Log2(num_slice_groups))
      }
      for (uint32_t i = 0; i <= pic_size_in_map_units_minus1; ++i) {
        uint32_t id = 0;
        if (bits > 0 && !br->ReadBits(bits, &id)) {  // slice_group_id[i]
          return false;
        }
      }
      return true;
    }
    case 1:
      return true;  // implicit map, no coded data
    default:
      return false;  // reserved / invalid
  }
}

// Skips a scaling matrix (transform_8x8-aware) as it appears in the PPS
// extension. Uses 64-bit intermediates so a malformed delta cannot overflow.
bool SkipScalingList(BitReader* br, uint32_t count) {
  int32_t last_scale = 8;
  int32_t next_scale = 8;
  for (uint32_t j = 0; j < count; ++j) {
    if (next_scale != 0) {
      int32_t delta = 0;
      if (!br->ReadSe(&delta)) {
        return false;
      }
      int64_t value = (static_cast<int64_t>(last_scale) + delta + 256) % 256;
      if (value < 0) {
        value += 256;
      }
      next_scale = static_cast<int32_t>(value);
    }
    if (next_scale != 0) {
      last_scale = next_scale;
    }
  }
  return true;
}

}  // namespace

bool ParsePps(const uint8_t* rbsp, size_t size, Pps* out) {
  if (rbsp == nullptr || out == nullptr) {
    return false;
  }
  BitReader br(rbsp, size);
  Pps pps;

  if (!br.ReadUe(&pps.pic_parameter_set_id) ||
      !br.ReadUe(&pps.seq_parameter_set_id) ||
      !br.ReadFlag(&pps.entropy_coding_mode_flag) ||
      !br.SkipBits(1)) {  // bottom_field_pic_order_in_frame_present_flag
    return false;
  }

  uint32_t num_slice_groups_minus1 = 0;
  if (!br.ReadUe(&num_slice_groups_minus1)) {
    return false;
  }
  if (num_slice_groups_minus1 > kMaxSliceGroupsMinus1) {
    return false;
  }
  if (num_slice_groups_minus1 > 0 &&
      !SkipSliceGroupMap(&br, num_slice_groups_minus1)) {
    return false;
  }

  int32_t se = 0;
  uint32_t bipred = 0;
  if (!br.ReadUe(&pps.num_ref_idx_l0_default_active_minus1) ||
      !br.ReadUe(&pps.num_ref_idx_l1_default_active_minus1) ||
      !br.SkipBits(1) ||           // weighted_pred_flag
      !br.ReadBits(2, &bipred) ||  // weighted_bipred_idc
      !br.ReadSe(&pps.pic_init_qp_minus26) ||
      !br.ReadSe(&se) ||  // pic_init_qs_minus26
      !br.ReadSe(&se) ||  // chroma_qp_index_offset
      !br.ReadFlag(&pps.deblocking_filter_control_present_flag) ||
      !br.SkipBits(1) ||  // constrained_intra_pred_flag
      !br.SkipBits(1)) {  // redundant_pic_cnt_present_flag
    return false;
  }

  // Optional extension (only present if RBSP data remains before the stop bit).
  if (br.MoreRbspData()) {
    if (!br.ReadFlag(&pps.transform_8x8_mode_flag)) {
      return false;
    }
    bool pic_scaling_matrix_present = false;
    if (!br.ReadFlag(&pic_scaling_matrix_present)) {
      return false;
    }
    if (pic_scaling_matrix_present) {
      const uint32_t lists = 6 + (pps.transform_8x8_mode_flag ? 2u : 0u);
      for (uint32_t i = 0; i < lists; ++i) {
        bool present = false;
        if (!br.ReadFlag(&present)) {
          return false;
        }
        if (present && !SkipScalingList(&br, i < 6 ? 16 : 64)) {
          return false;
        }
      }
    }
    if (!br.ReadSe(&se)) {  // second_chroma_qp_index_offset
      return false;
    }
  }

  *out = pps;
  return true;
}

}  // namespace v4l2wc::h264

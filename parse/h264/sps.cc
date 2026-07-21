// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h264/sps.h"

#include "parse/bit_reader.h"

namespace v4l2wc::h264 {
namespace {

bool IsHighProfile(uint8_t profile_idc) {
  switch (profile_idc) {
    case 100:
    case 110:
    case 122:
    case 244:
    case 44:
    case 83:
    case 86:
    case 118:
    case 128:
    case 138:
    case 139:
    case 134:
    case 135:
      return true;
    default:
      return false;
  }
}

// Reads and discards one scaling list (spec 7.3.2.1.1.1). Bounded by `count`.
// The running arithmetic uses int64 and a normalized modulo so a malformed
// (attacker-supplied) delta cannot overflow or leave next_scale out of 0..255.
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

bool ParseSps(const uint8_t* rbsp, size_t size, Sps* out) {
  if (rbsp == nullptr || out == nullptr) {
    return false;
  }
  BitReader br(rbsp, size);
  Sps sps;

  uint32_t profile = 0;
  uint32_t level = 0;
  if (!br.ReadBits(8, &profile) ||  // profile_idc
      !br.SkipBits(8) ||            // constraint flags + reserved
      !br.ReadBits(8, &level)) {    // level_idc
    return false;
  }
  sps.profile_idc = static_cast<uint8_t>(profile);
  sps.level_idc = static_cast<uint8_t>(level);

  if (!br.ReadUe(&sps.seq_parameter_set_id)) {
    return false;
  }

  bool separate_colour_plane = false;
  if (IsHighProfile(sps.profile_idc)) {
    if (!br.ReadUe(&sps.chroma_format_idc)) {
      return false;
    }
    if (sps.chroma_format_idc > 3) {
      return false;
    }
    if (sps.chroma_format_idc == 3 && !br.ReadFlag(&separate_colour_plane)) {
      return false;
    }
    uint32_t ignore = 0;
    if (!br.ReadUe(&ignore) ||  // bit_depth_luma_minus8
        !br.ReadUe(&ignore) ||  // bit_depth_chroma_minus8
        !br.SkipBits(1)) {      // qpprime_y_zero_transform_bypass_flag
      return false;
    }
    bool scaling_matrix_present = false;
    if (!br.ReadFlag(&scaling_matrix_present)) {
      return false;
    }
    if (scaling_matrix_present) {
      const uint32_t lists = (sps.chroma_format_idc != 3) ? 8 : 12;
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
  }

  uint32_t ignore = 0;
  if (!br.ReadUe(&ignore)) {  // log2_max_frame_num_minus4
    return false;
  }
  uint32_t pic_order_cnt_type = 0;
  if (!br.ReadUe(&pic_order_cnt_type)) {
    return false;
  }
  if (pic_order_cnt_type == 0) {
    if (!br.ReadUe(&ignore)) {  // log2_max_pic_order_cnt_lsb_minus4
      return false;
    }
  } else if (pic_order_cnt_type == 1) {
    int32_t sig = 0;
    uint32_t num_ref = 0;
    if (!br.SkipBits(1) ||       // delta_pic_order_always_zero_flag
        !br.ReadSe(&sig) ||      // offset_for_non_ref_pic
        !br.ReadSe(&sig) ||      // offset_for_top_to_bottom_field
        !br.ReadUe(&num_ref)) {  // num_ref_frames_in_pic_order_cnt_cycle
      return false;
    }
    if (num_ref > 255) {  // spec maximum; bound the loop
      return false;
    }
    for (uint32_t i = 0; i < num_ref; ++i) {
      if (!br.ReadSe(&sig)) {  // offset_for_ref_frame[i]
        return false;
      }
    }
  } else if (pic_order_cnt_type != 2) {
    return false;
  }

  uint32_t width_mbs_minus1 = 0;
  uint32_t height_map_units_minus1 = 0;
  if (!br.ReadUe(&ignore) ||            // max_num_ref_frames
      !br.SkipBits(1) ||                // gaps_in_frame_num_value_allowed
      !br.ReadUe(&width_mbs_minus1) ||  // pic_width_in_mbs_minus1
      !br.ReadUe(&height_map_units_minus1)) {  // pic_height_in_map_units_minus1
    return false;
  }
  if (!br.ReadFlag(&sps.frame_mbs_only_flag)) {
    return false;
  }
  if (!sps.frame_mbs_only_flag && !br.SkipBits(1)) {  // mb_adaptive_frame_field
    return false;
  }
  if (!br.SkipBits(1)) {  // direct_8x8_inference_flag
    return false;
  }

  // Bound the multiplications before deriving pixel dimensions.
  if (width_mbs_minus1 >= kMaxDimension / 16 ||
      height_map_units_minus1 >= kMaxDimension / 16) {
    return false;
  }
  uint32_t width = (width_mbs_minus1 + 1) * 16;
  uint32_t height = (2 - (sps.frame_mbs_only_flag ? 1u : 0u)) *
                    (height_map_units_minus1 + 1) * 16;

  bool frame_cropping = false;
  if (!br.ReadFlag(&frame_cropping)) {
    return false;
  }
  if (frame_cropping) {
    uint32_t crop_left = 0;
    uint32_t crop_right = 0;
    uint32_t crop_top = 0;
    uint32_t crop_bottom = 0;
    if (!br.ReadUe(&crop_left) || !br.ReadUe(&crop_right) ||
        !br.ReadUe(&crop_top) || !br.ReadUe(&crop_bottom)) {
      return false;
    }
    uint32_t crop_unit_x;
    uint32_t crop_unit_y;
    const uint32_t field_scale = sps.frame_mbs_only_flag ? 1u : 2u;
    if (sps.chroma_format_idc == 0 || separate_colour_plane) {
      crop_unit_x = 1;
      crop_unit_y = field_scale;
    } else {
      const uint32_t sub_width = (sps.chroma_format_idc == 3) ? 1u : 2u;
      const uint32_t sub_height = (sps.chroma_format_idc == 1) ? 2u : 1u;
      crop_unit_x = sub_width;
      crop_unit_y = sub_height * field_scale;
    }
    // Reject crops that would meet or exceed the coded size (underflow guard).
    const uint64_t dx =
        static_cast<uint64_t>(crop_left + crop_right) * crop_unit_x;
    const uint64_t dy =
        static_cast<uint64_t>(crop_top + crop_bottom) * crop_unit_y;
    if (dx >= width || dy >= height) {
      return false;
    }
    width -= static_cast<uint32_t>(dx);
    height -= static_cast<uint32_t>(dy);
  }

  if (width == 0 || height == 0 || width > kMaxDimension ||
      height > kMaxDimension) {
    return false;
  }
  sps.width = width;
  sps.height = height;
  *out = sps;
  return true;
}

}  // namespace v4l2wc::h264

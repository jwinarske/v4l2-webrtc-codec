// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/pps.h"

#include "parse/bit_reader.h"

namespace v4l2wc::h265 {

Pps::Pps() = default;

namespace {

// scaling_list_data (clause 7.3.4). Consumed and discarded; only correct
// advancement past the syntax matters here. Kept local to the translation unit,
// matching the H.264 parsers.
bool SkipScalingListData(BitReader* br) {
  for (uint32_t size_id = 0; size_id < 4; ++size_id) {
    for (uint32_t matrix_id = 0; matrix_id < 6;
         matrix_id += (size_id == 3) ? 3 : 1) {
      bool pred_mode = false;
      if (!br->ReadFlag(&pred_mode)) {
        return false;
      }
      if (!pred_mode) {
        uint32_t pred_matrix_id_delta = 0;
        if (!br->ReadUe(&pred_matrix_id_delta)) {
          return false;
        }
        continue;
      }
      uint32_t coef_num = 1u << (4 + (size_id << 1));
      if (coef_num > 64) {
        coef_num = 64;
      }
      int32_t ignore = 0;
      if (size_id > 1 && !br->ReadSe(&ignore)) {  // scaling_list_dc_coef_minus8
        return false;
      }
      for (uint32_t i = 0; i < coef_num; ++i) {
        if (!br->ReadSe(&ignore)) {  // scaling_list_delta_coef
          return false;
        }
      }
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

  if (!br.ReadUe(&pps.pps_pic_parameter_set_id) ||
      !br.ReadUe(&pps.pps_seq_parameter_set_id) ||
      !br.ReadFlag(&pps.dependent_slice_segments_enabled_flag) ||
      !br.ReadFlag(&pps.output_flag_present_flag) ||
      !br.ReadBits(3, &pps.num_extra_slice_header_bits) ||
      !br.ReadFlag(&pps.sign_data_hiding_enabled_flag) ||
      !br.ReadFlag(&pps.cabac_init_present_flag) ||
      !br.ReadUe(&pps.num_ref_idx_l0_default_active_minus1) ||
      !br.ReadUe(&pps.num_ref_idx_l1_default_active_minus1) ||
      !br.ReadSe(&pps.init_qp_minus26) ||
      !br.ReadFlag(&pps.constrained_intra_pred_flag) ||
      !br.ReadFlag(&pps.transform_skip_enabled_flag) ||
      !br.ReadFlag(&pps.cu_qp_delta_enabled_flag)) {
    return false;
  }
  if (pps.num_ref_idx_l0_default_active_minus1 > kMaxRefIdxDefault ||
      pps.num_ref_idx_l1_default_active_minus1 > kMaxRefIdxDefault) {
    return false;
  }
  if (pps.cu_qp_delta_enabled_flag && !br.ReadUe(&pps.diff_cu_qp_delta_depth)) {
    return false;
  }
  if (!br.ReadSe(&pps.pps_cb_qp_offset) || !br.ReadSe(&pps.pps_cr_qp_offset) ||
      !br.ReadFlag(&pps.pps_slice_chroma_qp_offsets_present_flag) ||
      !br.ReadFlag(&pps.weighted_pred_flag) ||
      !br.ReadFlag(&pps.weighted_bipred_flag) ||
      !br.ReadFlag(&pps.transquant_bypass_enabled_flag) ||
      !br.ReadFlag(&pps.tiles_enabled_flag) ||
      !br.ReadFlag(&pps.entropy_coding_sync_enabled_flag)) {
    return false;
  }

  if (pps.tiles_enabled_flag) {
    if (!br.ReadUe(&pps.num_tile_columns_minus1) ||
        !br.ReadUe(&pps.num_tile_rows_minus1)) {
      return false;
    }
    if (pps.num_tile_columns_minus1 >= kMaxTiles ||
        pps.num_tile_rows_minus1 >= kMaxTiles) {
      return false;
    }
    if (!br.ReadFlag(&pps.uniform_spacing_flag)) {
      return false;
    }
    if (!pps.uniform_spacing_flag) {
      // The last column / row width is not coded (it is derived from the
      // picture size), so only num_tile_columns_minus1 / num_tile_rows_minus1
      // explicit values are present.
      pps.column_width_minus1.resize(pps.num_tile_columns_minus1);
      pps.row_height_minus1.resize(pps.num_tile_rows_minus1);
      for (uint32_t i = 0; i < pps.num_tile_columns_minus1; ++i) {
        if (!br.ReadUe(&pps.column_width_minus1[i])) {
          return false;
        }
      }
      for (uint32_t i = 0; i < pps.num_tile_rows_minus1; ++i) {
        if (!br.ReadUe(&pps.row_height_minus1[i])) {
          return false;
        }
      }
    }
    if (!br.ReadFlag(&pps.loop_filter_across_tiles_enabled_flag)) {
      return false;
    }
  }

  if (!br.ReadFlag(&pps.pps_loop_filter_across_slices_enabled_flag) ||
      !br.ReadFlag(&pps.deblocking_filter_control_present_flag)) {
    return false;
  }
  if (pps.deblocking_filter_control_present_flag) {
    if (!br.ReadFlag(&pps.deblocking_filter_override_enabled_flag) ||
        !br.ReadFlag(&pps.pps_deblocking_filter_disabled_flag)) {
      return false;
    }
    if (!pps.pps_deblocking_filter_disabled_flag) {
      if (!br.ReadSe(&pps.pps_beta_offset_div2) ||
          !br.ReadSe(&pps.pps_tc_offset_div2)) {
        return false;
      }
    }
  }

  if (!br.ReadFlag(&pps.pps_scaling_list_data_present_flag)) {
    return false;
  }
  if (pps.pps_scaling_list_data_present_flag && !SkipScalingListData(&br)) {
    return false;
  }

  if (!br.ReadFlag(&pps.lists_modification_present_flag) ||
      !br.ReadUe(&pps.log2_parallel_merge_level_minus2) ||
      !br.ReadFlag(&pps.slice_segment_header_extension_present_flag)) {
    return false;
  }

  // Everything past here (pps_extension flags, VUI-independent extensions) is
  // not needed by the decoder or the slice-segment header, so the parse stops.

  *out = pps;
  return true;
}

}  // namespace v4l2wc::h265

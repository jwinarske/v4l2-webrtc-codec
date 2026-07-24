// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/pps.h"

#include "parse/bit_reader.h"
#include "parse/h265/scaling_list.h"

namespace v4l2wc::h265 {

Pps::Pps() = default;

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
  if (pps.pps_scaling_list_data_present_flag &&
      !ParseScalingListData(&br, &pps.scaling_list)) {
    return false;
  }

  if (!br.ReadFlag(&pps.lists_modification_present_flag) ||
      !br.ReadUe(&pps.log2_parallel_merge_level_minus2) ||
      !br.ReadFlag(&pps.slice_segment_header_extension_present_flag)) {
    return false;
  }

  bool pps_extension_present = false;
  if (!br.ReadFlag(&pps_extension_present)) {
    return false;
  }
  if (pps_extension_present) {
    bool pps_multilayer_extension = false;
    bool pps_3d_extension = false;
    bool pps_scc_extension = false;
    uint32_t pps_extension_4bits = 0;
    if (!br.ReadFlag(&pps.pps_range_extension_flag) ||
        !br.ReadFlag(&pps_multilayer_extension) ||
        !br.ReadFlag(&pps_3d_extension) || !br.ReadFlag(&pps_scc_extension) ||
        !br.ReadBits(4, &pps_extension_4bits)) {
      return false;
    }
    if (pps.pps_range_extension_flag) {
      PpsRangeExtension& rx = pps.range_extension;
      if (pps.transform_skip_enabled_flag) {
        uint32_t log2_max_ts_minus2 = 0;
        if (!br.ReadUe(&log2_max_ts_minus2) || log2_max_ts_minus2 > 3) {
          return false;  // block size 4..32 (CTB-bounded)
        }
        rx.log2_max_transform_skip_block_size = log2_max_ts_minus2 + 2;
      }
      if (!br.ReadFlag(&rx.cross_component_prediction_enabled_flag) ||
          !br.ReadFlag(&rx.chroma_qp_offset_list_enabled_flag)) {
        return false;
      }
      if (rx.chroma_qp_offset_list_enabled_flag) {
        if (!br.ReadUe(&rx.diff_cu_chroma_qp_offset_depth) ||
            !br.ReadUe(&rx.chroma_qp_offset_list_len_minus1) ||
            rx.chroma_qp_offset_list_len_minus1 > 5) {  // list holds 6 entries
          return false;
        }
        for (uint32_t i = 0; i <= rx.chroma_qp_offset_list_len_minus1; ++i) {
          if (!br.ReadSe(&rx.cb_qp_offset_list[i]) ||
              !br.ReadSe(&rx.cr_qp_offset_list[i])) {
            return false;
          }
        }
      }
      if (!br.ReadUe(&rx.log2_sao_offset_scale_luma) ||
          !br.ReadUe(&rx.log2_sao_offset_scale_chroma) ||
          rx.log2_sao_offset_scale_luma > 6 ||
          rx.log2_sao_offset_scale_chroma > 6) {
        return false;
      }
    }
    // The multilayer / 3D / SCC / reserved extensions are not parsed.
  }

  *out = pps;
  return true;
}

}  // namespace v4l2wc::h265

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.265 PPS parser. Real PPS NALs from ffmpeg/libx265 prove
// the parser is encoder-conformant on the common path; a synthetic PPS built
// bit-for-bit exercises the tiles and deblocking-control branches that the
// libx265 clips leave off. No framework: a failing check sets the exit code.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h265/nal.h"
#include "parse/h265/pps.h"

using v4l2wc::h265::ParseAnnexB;
using v4l2wc::h265::ParsePps;
using v4l2wc::h265::Pps;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

static bool ParsePpsFromNal(const std::vector<uint8_t>& nal, Pps* out) {
  std::vector<uint8_t> buf = {0x00, 0x00, 0x01};
  buf.insert(buf.end(), nal.begin(), nal.end());
  auto nals = ParseAnnexB(buf.data(), buf.size());
  if (nals.size() != 1) {
    return false;
  }
  return ParsePps(nals[0].rbsp.data(), nals[0].rbsp.size(), out);
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
  void WriteSe(int32_t v) {
    // se(v): 0,1,-1,2,-2,... -> ue code 0,1,2,3,4,...
    uint32_t code = v <= 0 ? static_cast<uint32_t>(-2 * v)
                           : static_cast<uint32_t>(2 * v - 1);
    WriteUe(code);
  }
  const std::vector<uint8_t>& bytes() const { return bytes_; }

 private:
  std::vector<uint8_t> bytes_;
  uint32_t bit_ = 0;
};

static std::vector<uint8_t> BuildSyntheticPps() {
  BitWriter w;
  w.WriteUe(2);        // pps_pic_parameter_set_id
  w.WriteUe(1);        // pps_seq_parameter_set_id
  w.WriteFlag(true);   // dependent_slice_segments_enabled_flag
  w.WriteFlag(false);  // output_flag_present_flag
  w.WriteBits(2, 3);   // num_extra_slice_header_bits
  w.WriteFlag(false);  // sign_data_hiding_enabled_flag
  w.WriteFlag(true);   // cabac_init_present_flag
  w.WriteUe(3);        // num_ref_idx_l0_default_active_minus1
  w.WriteUe(1);        // num_ref_idx_l1_default_active_minus1
  w.WriteSe(-4);       // init_qp_minus26
  w.WriteFlag(false);  // constrained_intra_pred_flag
  w.WriteFlag(true);   // transform_skip_enabled_flag
  w.WriteFlag(true);   // cu_qp_delta_enabled_flag
  w.WriteUe(2);        // diff_cu_qp_delta_depth
  w.WriteSe(-3);       // pps_cb_qp_offset
  w.WriteSe(5);        // pps_cr_qp_offset
  w.WriteFlag(true);   // pps_slice_chroma_qp_offsets_present_flag
  w.WriteFlag(true);   // weighted_pred_flag
  w.WriteFlag(true);   // weighted_bipred_flag
  w.WriteFlag(false);  // transquant_bypass_enabled_flag
  w.WriteFlag(true);   // tiles_enabled_flag
  w.WriteFlag(true);   // entropy_coding_sync_enabled_flag
  w.WriteUe(2);        // num_tile_columns_minus1
  w.WriteUe(1);        // num_tile_rows_minus1
  w.WriteFlag(false);  // uniform_spacing_flag
  w.WriteUe(3);        // column_width_minus1[0]
  w.WriteUe(4);        // column_width_minus1[1]
  w.WriteUe(5);        // row_height_minus1[0]
  w.WriteFlag(true);   // loop_filter_across_tiles_enabled_flag
  w.WriteFlag(true);   // pps_loop_filter_across_slices_enabled_flag
  w.WriteFlag(true);   // deblocking_filter_control_present_flag
  w.WriteFlag(true);   // deblocking_filter_override_enabled_flag
  w.WriteFlag(false);  // pps_deblocking_filter_disabled_flag
  w.WriteSe(-2);       // pps_beta_offset_div2
  w.WriteSe(3);        // pps_tc_offset_div2
  w.WriteFlag(false);  // pps_scaling_list_data_present_flag
  w.WriteFlag(true);   // lists_modification_present_flag
  w.WriteUe(1);        // log2_parallel_merge_level_minus2
  w.WriteFlag(true);   // slice_segment_header_extension_present_flag
  return w.bytes();
}

int main() {
  // ---- Real libx265 PPS vectors. Both use WPP, weighted-pred, cu-qp-delta,
  // sign-data-hiding, no tiles; they differ in the chroma QP offsets. ----
  {
    Pps pps;
    // 4:2:2 clip: chroma QP offsets 0.
    const bool ok =
        ParsePpsFromNal({0x44, 0x01, 0xc1, 0x72, 0xb4, 0x62, 0x40}, &pps);
    CHECK(ok);
    CHECK(pps.pps_pic_parameter_set_id == 0);
    CHECK(pps.pps_seq_parameter_set_id == 0);
    CHECK(pps.sign_data_hiding_enabled_flag == true);
    CHECK(pps.cu_qp_delta_enabled_flag == true);
    CHECK(pps.weighted_pred_flag == true);
    CHECK(pps.weighted_bipred_flag == false);
    CHECK(pps.tiles_enabled_flag == false);
    CHECK(pps.entropy_coding_sync_enabled_flag == true);
    CHECK(pps.pps_cb_qp_offset == 0);
    CHECK(pps.pps_cr_qp_offset == 0);
    CHECK(pps.deblocking_filter_control_present_flag == false);
    CHECK(pps.slice_segment_header_extension_present_flag == false);
  }
  {
    Pps pps;
    // 1080p clip: chroma QP offsets 6.
    const bool ok =
        ParsePpsFromNal({0x44, 0x01, 0xc1, 0x72, 0x86, 0x0c, 0x46, 0x24}, &pps);
    CHECK(ok);
    CHECK(pps.sign_data_hiding_enabled_flag == true);
    CHECK(pps.cu_qp_delta_enabled_flag == true);
    CHECK(pps.entropy_coding_sync_enabled_flag == true);
    CHECK(pps.pps_cb_qp_offset == 6);
    CHECK(pps.pps_cr_qp_offset == 6);
    CHECK(pps.tiles_enabled_flag == false);
  }

  // Truncation and null inputs are rejected, not over-read.
  {
    Pps pps;
    CHECK(!ParsePpsFromNal({0x44, 0x01, 0xc1}, &pps));
    CHECK(!ParsePps(nullptr, 4, &pps));
    const uint8_t dummy = 0;
    CHECK(!ParsePps(&dummy, 1, nullptr));
  }

  // ---- Synthetic PPS: tiles + deblocking-control branches. ----
  {
    const std::vector<uint8_t> rbsp = BuildSyntheticPps();
    Pps pps;
    const bool ok = ParsePps(rbsp.data(), rbsp.size(), &pps);
    CHECK(ok);
    if (ok) {
      CHECK(pps.pps_pic_parameter_set_id == 2);
      CHECK(pps.pps_seq_parameter_set_id == 1);
      CHECK(pps.dependent_slice_segments_enabled_flag == true);
      CHECK(pps.output_flag_present_flag == false);
      CHECK(pps.num_extra_slice_header_bits == 2);
      CHECK(pps.sign_data_hiding_enabled_flag == false);
      CHECK(pps.cabac_init_present_flag == true);
      CHECK(pps.num_ref_idx_l0_default_active_minus1 == 3);
      CHECK(pps.num_ref_idx_l1_default_active_minus1 == 1);
      CHECK(pps.init_qp_minus26 == -4);
      CHECK(pps.transform_skip_enabled_flag == true);
      CHECK(pps.cu_qp_delta_enabled_flag == true);
      CHECK(pps.diff_cu_qp_delta_depth == 2);
      CHECK(pps.pps_cb_qp_offset == -3);
      CHECK(pps.pps_cr_qp_offset == 5);
      CHECK(pps.pps_slice_chroma_qp_offsets_present_flag == true);
      CHECK(pps.weighted_pred_flag == true);
      CHECK(pps.weighted_bipred_flag == true);
      CHECK(pps.tiles_enabled_flag == true);
      CHECK(pps.entropy_coding_sync_enabled_flag == true);
      CHECK(pps.num_tile_columns_minus1 == 2);
      CHECK(pps.num_tile_rows_minus1 == 1);
      CHECK(pps.uniform_spacing_flag == false);
      // Explicit tile geometry: two coded column widths and one row height.
      CHECK(pps.column_width_minus1.size() == 2);
      CHECK(pps.column_width_minus1[0] == 3);
      CHECK(pps.column_width_minus1[1] == 4);
      CHECK(pps.row_height_minus1.size() == 1);
      CHECK(pps.row_height_minus1[0] == 5);
      CHECK(pps.loop_filter_across_tiles_enabled_flag == true);
      CHECK(pps.pps_loop_filter_across_slices_enabled_flag == true);
      CHECK(pps.deblocking_filter_control_present_flag == true);
      CHECK(pps.deblocking_filter_override_enabled_flag == true);
      CHECK(pps.pps_deblocking_filter_disabled_flag == false);
      CHECK(pps.pps_beta_offset_div2 == -2);
      CHECK(pps.pps_tc_offset_div2 == 3);
      CHECK(pps.pps_scaling_list_data_present_flag == false);
      CHECK(pps.lists_modification_present_flag == true);
      CHECK(pps.log2_parallel_merge_level_minus2 == 1);
      CHECK(pps.slice_segment_header_extension_present_flag == true);
    }
  }

  // num_ref_idx over the spec maximum is rejected.
  {
    BitWriter w;
    w.WriteUe(0);        // pps_pic_parameter_set_id
    w.WriteUe(0);        // pps_seq_parameter_set_id
    w.WriteFlag(false);  // dependent_slice_segments_enabled_flag
    w.WriteFlag(false);  // output_flag_present_flag
    w.WriteBits(0, 3);   // num_extra_slice_header_bits
    w.WriteFlag(false);  // sign_data_hiding_enabled_flag
    w.WriteFlag(false);  // cabac_init_present_flag
    w.WriteUe(15);       // num_ref_idx_l0_default_active_minus1 (> 14)
    w.WriteUe(0);        // num_ref_idx_l1_default_active_minus1
    Pps pps;
    CHECK(!ParsePps(w.bytes().data(), w.bytes().size(), &pps));
  }

  if (g_failures == 0) {
    std::printf("H265_PPS_TEST_OK\n");
    return 0;
  }
  std::printf("H265_PPS_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

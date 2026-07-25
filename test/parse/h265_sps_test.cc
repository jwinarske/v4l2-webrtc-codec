// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.265 SPS parser. Two kinds of vectors:
//   1. Real SPS NALs emitted by ffmpeg/libx265 at known resolutions and chroma
//      formats, checked end-to-end through ParseAnnexB + ParseSps. These prove
//      the parser is encoder-conformant on geometry, profile/tier/level, and
//      the CTB derivation.
//   2. A synthetic SPS built bit-for-bit with a BitWriter, exercising the
//      short-term reference-picture-set derivation (including the inter-
//      predicted form), which libx265 signals in slice headers rather than the
//      SPS. The expected delta-POC lists are computed by hand from
//      clause 7.4.8.
// No framework: a failing check prints and sets the exit code.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h265/nal.h"
#include "parse/h265/sps.h"

using v4l2wc::h265::ParseAnnexB;
using v4l2wc::h265::ParseSps;
using v4l2wc::h265::Sps;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

// Parses the SPS out of a raw NAL (2-byte header + emulation bytes intact, as
// dumped from a stream) by wrapping it in a start code and running the real
// Annex-B splitter first.
static bool ParseSpsFromNal(const std::vector<uint8_t>& nal, Sps* out) {
  std::vector<uint8_t> buf = {0x00, 0x00, 0x01};
  buf.insert(buf.end(), nal.begin(), nal.end());
  auto nals = ParseAnnexB(buf.data(), buf.size());
  if (nals.size() != 1) {
    return false;
  }
  return ParseSps(nals[0].rbsp.data(), nals[0].rbsp.size(), out);
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
    WriteBits(0, nbits - 1);  // leading zeros
    WriteBits(code, nbits);   // the value+1 itself
  }
  const std::vector<uint8_t>& bytes() const { return bytes_; }

 private:
  std::vector<uint8_t> bytes_;
  uint32_t bit_ = 0;  // 0..7, next bit position within the last byte
};

// Builds a 64x64 4:2:0 SPS with three short-term RPSs: two explicit and one
// inter-predicted from the second.
static std::vector<uint8_t> BuildSyntheticSps() {
  BitWriter w;
  w.WriteBits(0, 4);  // sps_video_parameter_set_id
  w.WriteBits(0, 3);  // sps_max_sub_layers_minus1
  w.WriteFlag(true);  // sps_temporal_id_nesting_flag
  // profile_tier_level(1, 0):
  w.WriteBits(0, 2);   // general_profile_space
  w.WriteFlag(false);  // general_tier_flag
  w.WriteBits(1, 5);   // general_profile_idc = 1 (Main)
  w.WriteBits(0, 32);  // 32 compatibility flags (skipped by parser)
  w.WriteBits(0, 32);  // 4 source + 28 constraint flags
  w.WriteBits(0, 16);  // remaining 15 constraint + 1 reserved (total 80)
  w.WriteBits(30, 8);  // general_level_idc = 30 (level 1.0)

  w.WriteUe(0);        // sps_seq_parameter_set_id
  w.WriteUe(1);        // chroma_format_idc = 4:2:0
  w.WriteUe(64);       // pic_width_in_luma_samples
  w.WriteUe(64);       // pic_height_in_luma_samples
  w.WriteFlag(false);  // conformance_window_flag
  w.WriteUe(0);        // bit_depth_luma_minus8
  w.WriteUe(0);        // bit_depth_chroma_minus8
  w.WriteUe(0);        // log2_max_pic_order_cnt_lsb_minus4 -> 4
  w.WriteFlag(true);   // sps_sub_layer_ordering_info_present_flag
  w.WriteUe(4);        // sps_max_dec_pic_buffering_minus1[0]
  w.WriteUe(0);        // sps_max_num_reorder_pics[0]
  w.WriteUe(0);        // sps_max_latency_increase_plus1[0]
  w.WriteUe(0);        // log2_min_luma_coding_block_size_minus3 -> 3
  w.WriteUe(3);        // log2_diff_max_min_luma_coding_block_size -> ctb 6
  w.WriteUe(0);        // log2_min_luma_transform_block_size_minus2
  w.WriteUe(3);        // log2_diff_max_min_luma_transform_block_size
  w.WriteUe(0);        // max_transform_hierarchy_depth_inter
  w.WriteUe(0);        // max_transform_hierarchy_depth_intra
  w.WriteFlag(false);  // scaling_list_enabled_flag
  w.WriteFlag(false);  // amp_enabled_flag
  w.WriteFlag(true);   // sample_adaptive_offset_enabled_flag
  w.WriteFlag(false);  // pcm_enabled_flag

  w.WriteUe(3);  // num_short_term_ref_pic_sets

  // RPS[0], explicit: neg = {-1, -3} used{1,1}, pos = {+2} used{1}.
  w.WriteUe(2);       // num_negative_pics
  w.WriteUe(1);       // num_positive_pics
  w.WriteUe(0);       // delta_poc_s0_minus1[0] -> -1
  w.WriteFlag(true);  // used_by_curr_pic_s0_flag[0]
  w.WriteUe(1);       // delta_poc_s0_minus1[1] -> -3
  w.WriteFlag(true);  // used_by_curr_pic_s0_flag[1]
  w.WriteUe(1);       // delta_poc_s1_minus1[0] -> +2
  w.WriteFlag(true);  // used_by_curr_pic_s1_flag[0]

  // RPS[1], explicit: neg = {-1} used{1}, pos = {}.
  w.WriteFlag(false);  // inter_ref_pic_set_prediction_flag
  w.WriteUe(1);        // num_negative_pics
  w.WriteUe(0);        // num_positive_pics
  w.WriteUe(0);        // delta_poc_s0_minus1[0] -> -1
  w.WriteFlag(true);   // used_by_curr_pic_s0_flag[0]

  // RPS[2], inter-predicted from RPS[1] (RefRpsIdx = 1), deltaRps = +1.
  w.WriteFlag(true);   // inter_ref_pic_set_prediction_flag
  w.WriteFlag(false);  // delta_rps_sign (positive)
  w.WriteUe(0);        // abs_delta_rps_minus1 -> deltaRps = +1
  // j = 0..NumDeltaPocs[1] = 0..1:
  w.WriteFlag(true);   // used_by_curr_pic_flag[0] (use_delta implied 1)
  w.WriteFlag(false);  // used_by_curr_pic_flag[1]
  w.WriteFlag(true);   // use_delta_flag[1]

  w.WriteFlag(false);  // long_term_ref_pics_present_flag
  w.WriteFlag(true);   // sps_temporal_mvp_enabled_flag
  w.WriteFlag(true);   // strong_intra_smoothing_enabled_flag
  w.WriteFlag(false);  // vui_parameters_present_flag (no VUI)
  w.WriteFlag(true);   // sps_extension_present_flag
  w.WriteFlag(true);   // sps_range_extension_flag
  w.WriteFlag(false);  // sps_multilayer_extension_flag
  w.WriteFlag(false);  // sps_3d_extension_flag
  w.WriteFlag(false);  // sps_scc_extension_flag
  w.WriteBits(0, 4);   // sps_extension_4bits
  // sps_range_extension: nine flags, a recognisable pattern.
  w.WriteFlag(true);   // transform_skip_rotation_enabled_flag
  w.WriteFlag(false);  // transform_skip_context_enabled_flag
  w.WriteFlag(true);   // implicit_rdpcm_enabled_flag
  w.WriteFlag(false);  // explicit_rdpcm_enabled_flag
  w.WriteFlag(false);  // extended_precision_processing_flag
  w.WriteFlag(true);   // intra_smoothing_disabled_flag
  w.WriteFlag(true);   // high_precision_offsets_enabled_flag
  w.WriteFlag(false);  // persistent_rice_adaptation_enabled_flag
  w.WriteFlag(false);  // cabac_bypass_alignment_enabled_flag
  return w.bytes();
}

int main() {
  // ---- Real ffmpeg/libx265 SPS vectors (geometry + PTL). ----
  struct Vec {
    const char* name;
    std::vector<uint8_t> nal;
    uint32_t width, height, chroma;
  };
  const std::vector<Vec> vecs = {
      {"hd",
       {0x42, 0x01, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9e, 0x08,
        0x00, 0x00, 0x03, 0x00, 0x00, 0x5d, 0x90, 0x00, 0x50, 0x10, 0x05,
        0xa2, 0xcb, 0x2b, 0x34, 0x92, 0x65, 0x78, 0x0b, 0x70, 0x20, 0x20,
        0x00, 0x40, 0x00, 0x00, 0x03, 0x00, 0x40, 0x00, 0x00, 0x06, 0x42},
       1280,
       720,
       3},
      {"fhd",
       {0x42, 0x01, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9e, 0x08, 0x00,
        0x00, 0x03, 0x00, 0x00, 0x78, 0x90, 0x00, 0x78, 0x10, 0x02, 0x1c, 0xb2,
        0xca, 0xcd, 0x24, 0x99, 0x5e, 0x02, 0xdc, 0x08, 0x08, 0x00, 0x10, 0x00,
        0x00, 0x03, 0x00, 0x10, 0x00, 0x00, 0x03, 0x01, 0x90, 0x80},
       1920,
       1080,
       3},
      {"sd",
       {0x42, 0x01, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9e, 0x08, 0x00,
        0x00, 0x03, 0x00, 0x00, 0x5a, 0x90, 0x00, 0xa0, 0x40, 0x3c, 0x2c, 0xb2,
        0xb3, 0x49, 0x26, 0x57, 0x80, 0xb7, 0x02, 0x02, 0x00, 0x04, 0x00, 0x00,
        0x03, 0x00, 0x04, 0x00, 0x00, 0x03, 0x00, 0x64, 0x20},
       640,
       480,
       3},
      {"chroma422",
       {0x42, 0x01, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9d,
        0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x3c, 0xb0, 0x0b, 0x08,
        0x04, 0x85, 0x96, 0x56, 0x69, 0x24, 0xca, 0xf0, 0x16, 0x80,
        0x80, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x84},
       352,
       288,
       2},
  };
  for (const auto& v : vecs) {
    Sps sps;
    const bool ok = ParseSpsFromNal(v.nal, &sps);
    CHECK(ok);
    if (!ok) {
      std::printf("  (%s failed to parse)\n", v.name);
      continue;
    }
    CHECK(sps.width == v.width);
    CHECK(sps.height == v.height);
    CHECK(sps.chroma_format_idc == v.chroma);
    CHECK(sps.bit_depth_luma == 8);
    CHECK(sps.bit_depth_chroma == 8);
    // These clips share a profile/tier and 64-sample CTBs.
    CHECK(sps.general_tier_flag == false);
    CHECK(sps.log2_ctb_size == 6);
    CHECK(sps.log2_min_cb_size == 3);
    CHECK(sps.sample_adaptive_offset_enabled_flag == true);
    // The CTB grid ceils the coded dimensions.
    CHECK(sps.pic_width_in_ctbs == (sps.pic_width_in_luma_samples + 63) / 64);
    CHECK(sps.pic_height_in_ctbs == (sps.pic_height_in_luma_samples + 63) / 64);
    CHECK(sps.pic_size_in_ctbs ==
          sps.pic_width_in_ctbs * sps.pic_height_in_ctbs);
  }

  // A real Main 10 SPS (libx265, 640x480, 10-bit 4:2:0): the bit depths drive
  // the decoder's profile and surface-format choice.
  {
    Sps sps;
    const bool ok = ParseSpsFromNal(
        {0x42, 0x01, 0x01, 0x02, 0x20, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00,
         0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5a, 0xa0, 0x05, 0x02, 0x01,
         0xe1, 0x36, 0x59, 0x59, 0xa4, 0x93, 0x2b, 0xc0, 0x5a, 0x02, 0x00,
         0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x32, 0x10},
        &sps);
    CHECK(ok);
    CHECK(sps.width == 640);
    CHECK(sps.height == 480);
    CHECK(sps.chroma_format_idc == 1);  // 4:2:0
    CHECK(sps.bit_depth_luma == 10);
    CHECK(sps.bit_depth_chroma == 10);
    CHECK(sps.general_profile_idc == 2);  // Main 10
  }

  // A real SPS with a VUI color description (libx265, colorprim/transfer/
  // colormatrix=bt709, range=limited): the video_signal_type is captured so a
  // presentation layer can convert YUV to RGB with the right coefficients.
  {
    Sps sps;
    const bool ok = ParseSpsFromNal(
        {0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00,
         0x03, 0x00, 0x00, 0x03, 0x00, 0x3c, 0xa0, 0x0a, 0x08, 0x0f, 0x16, 0x59,
         0x59, 0xa4, 0x93, 0x2b, 0xc0, 0x5a, 0x80, 0x80, 0x80, 0x82, 0x00, 0x00,
         0x03, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x32, 0x10, 0x00},
        &sps);
    CHECK(ok);
    CHECK(sps.colour_description_present_flag);
    CHECK(sps.colour_primaries == 1);          // BT.709
    CHECK(sps.transfer_characteristics == 1);  // BT.709
    CHECK(sps.matrix_coeffs == 1);             // BT.709
    CHECK(!sps.video_full_range_flag);         // limited (studio) range
  }

  // Truncating a real SPS must fail rather than read past the buffer.
  {
    Sps sps;
    std::vector<uint8_t> partial(vecs[0].nal.begin(), vecs[0].nal.begin() + 6);
    CHECK(!ParseSpsFromNal(partial, &sps));
  }

  // A null buffer or null out is rejected.
  {
    Sps sps;
    CHECK(!ParseSps(nullptr, 10, &sps));
    const uint8_t dummy = 0;
    CHECK(!ParseSps(&dummy, 1, nullptr));
  }

  // ---- Synthetic SPS: short-term RPS derivation. ----
  {
    const std::vector<uint8_t> rbsp = BuildSyntheticSps();
    Sps sps;
    const bool ok = ParseSps(rbsp.data(), rbsp.size(), &sps);
    CHECK(ok);
    if (ok) {
      CHECK(sps.width == 64);
      CHECK(sps.height == 64);
      CHECK(sps.chroma_format_idc == 1);
      CHECK(sps.general_profile_idc == 1);
      CHECK(sps.general_level_idc == 30);
      CHECK(sps.log2_max_pic_order_cnt_lsb == 4);
      CHECK(sps.log2_ctb_size == 6);
      CHECK(sps.pic_width_in_ctbs == 1);
      CHECK(sps.pic_size_in_ctbs == 1);
      CHECK(sps.sps_max_dec_pic_buffering_minus1 == 4);
      CHECK(sps.sample_adaptive_offset_enabled_flag == true);
      CHECK(sps.amp_enabled_flag == false);
      CHECK(sps.pcm_enabled_flag == false);
      CHECK(sps.long_term_ref_pics_present_flag == false);
      CHECK(sps.sps_temporal_mvp_enabled_flag == true);
      CHECK(sps.strong_intra_smoothing_enabled_flag == true);
      // The range extension is reached only if the VUI (absent here) and the
      // extension flags are parsed correctly.
      CHECK(sps.sps_range_extension_flag == true);
      CHECK(sps.range_extension.transform_skip_rotation_enabled_flag == true);
      CHECK(sps.range_extension.transform_skip_context_enabled_flag == false);
      CHECK(sps.range_extension.implicit_rdpcm_enabled_flag == true);
      CHECK(sps.range_extension.explicit_rdpcm_enabled_flag == false);
      CHECK(sps.range_extension.extended_precision_processing_flag == false);
      CHECK(sps.range_extension.intra_smoothing_disabled_flag == true);
      CHECK(sps.range_extension.high_precision_offsets_enabled_flag == true);
      CHECK(sps.range_extension.persistent_rice_adaptation_enabled_flag ==
            false);
      CHECK(sps.range_extension.cabac_bypass_alignment_enabled_flag == false);

      CHECK(sps.short_term_rps.size() == 3);
      if (sps.short_term_rps.size() == 3) {
        const auto& r0 = sps.short_term_rps[0];
        CHECK(r0.num_negative_pics == 2);
        CHECK(r0.num_positive_pics == 1);
        CHECK(r0.num_delta_pocs == 3);
        CHECK(r0.delta_poc_s0[0] == -1);
        CHECK(r0.delta_poc_s0[1] == -3);
        CHECK(r0.used_s0[0] && r0.used_s0[1]);
        CHECK(r0.delta_poc_s1[0] == 2);
        CHECK(r0.used_s1[0]);

        const auto& r1 = sps.short_term_rps[1];
        CHECK(r1.num_negative_pics == 1);
        CHECK(r1.num_positive_pics == 0);
        CHECK(r1.num_delta_pocs == 1);
        CHECK(r1.delta_poc_s0[0] == -1);

        // RPS[2] inter-predicted from RPS[1], deltaRps = +1:
        // negative side collapses (−1 + 1 = 0, not < 0) and the deltaRps entry
        // lands on the positive side as +1, unused.
        const auto& r2 = sps.short_term_rps[2];
        CHECK(r2.num_negative_pics == 0);
        CHECK(r2.num_positive_pics == 1);
        CHECK(r2.num_delta_pocs == 1);
        CHECK(r2.delta_poc_s1[0] == 1);
        CHECK(r2.used_s1[0] == false);
      }
    }
  }

  if (g_failures == 0) {
    std::printf("H265_SPS_TEST_OK\n");
    return 0;
  }
  std::printf("H265_SPS_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

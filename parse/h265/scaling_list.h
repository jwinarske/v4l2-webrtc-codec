// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.265 / HEVC scaling_list_data parsing (clause 7.3.4). The SPS and PPS both
// carry this optional structure, so it lives here and both parsers share it.
//
// The coefficients are decoded into raster order (the up-right diagonal scan of
// clause 6.5.3 is applied while placing the delta-coded values), which is the
// layout a stateless decoder's inverse-quantisation matrix wants. Attacker-
// controlled input: every field is read through the bounds-checked BitReader.
#ifndef V4L2WC_PARSE_H265_SCALING_LIST_H_
#define V4L2WC_PARSE_H265_SCALING_LIST_H_

#include <cstdint>

#include "parse/bit_reader.h"

namespace v4l2wc::h265 {

using v4l2wc::BitReader;

// Decoded scaling lists, in raster order. The 16x16 and 32x32 lists hold the
// 64 coded coefficients (an 8x8 low-frequency grid) plus a separate DC value.
// MatrixId 0..2 are intra, 3..5 inter; the 32x32 size has only matrices 0 and 1
// (from matrixId 0 and 3).
struct ScalingListData {
  uint8_t list4x4[6][16] = {};
  uint8_t list8x8[6][64] = {};
  uint8_t list16x16[6][64] = {};
  uint8_t list32x32[2][64] = {};
  uint8_t dc16x16[6] = {};
  uint8_t dc32x32[2] = {};
};

// Fills *out with the HEVC default scaling lists (Tables 7-5 / 7-6). Used when
// scaling lists are enabled but no scaling_list_data is signalled.
void SetDefaultScalingList(ScalingListData* out);

// Parses scaling_list_data (clause 7.3.4) into *out, resolving default and
// copy-from-earlier lists. Returns false on malformed input.
bool ParseScalingListData(BitReader* br, ScalingListData* out);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_SCALING_LIST_H_

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/scaling_list.h"

#include <cstring>

namespace v4l2wc::h265 {

namespace {

// Default scaling lists (Tables 7-5 / 7-6), in raster order. The 4x4 list is
// flat 16; the 8x8 intra/inter grids also seed the 16x16 and 32x32 sizes.
constexpr uint8_t kDefault4x4[16] = {16, 16, 16, 16, 16, 16, 16, 16,
                                     16, 16, 16, 16, 16, 16, 16, 16};

constexpr uint8_t kDefaultIntra8x8[64] = {
    16, 16, 16, 16, 17, 18, 21, 24, 16, 16, 16, 16, 17, 19, 22, 25,
    16, 16, 17, 18, 20, 22, 25, 29, 16, 16, 18, 21, 24, 27, 31, 36,
    17, 17, 20, 24, 30, 35, 41, 47, 18, 19, 22, 27, 35, 44, 54, 65,
    21, 22, 25, 31, 41, 54, 70, 88, 24, 25, 29, 36, 47, 65, 88, 115};

constexpr uint8_t kDefaultInter8x8[64] = {
    16, 16, 16, 16, 17, 18, 20, 24, 16, 16, 16, 17, 18, 20, 24, 25,
    16, 16, 17, 18, 20, 24, 25, 28, 16, 17, 18, 20, 24, 25, 28, 33,
    17, 18, 20, 24, 25, 28, 33, 41, 18, 20, 24, 25, 28, 33, 41, 54,
    20, 24, 25, 28, 33, 41, 54, 71, 24, 25, 28, 33, 41, 54, 71, 91};

// The coefficient array for one (sizeId, matrixId). sizeId 3 has only two
// matrices, indexed by matrixId / 3 (matrixId is 0 or 3).
uint8_t* CoefList(ScalingListData* s, int size_id, int matrix_id) {
  switch (size_id) {
    case 0:
      return s->list4x4[matrix_id];
    case 1:
      return s->list8x8[matrix_id];
    case 2:
      return s->list16x16[matrix_id];
    default:
      return s->list32x32[matrix_id / 3];
  }
}

void SetDc(ScalingListData* s, int size_id, int matrix_id, uint8_t value) {
  if (size_id == 2) {
    s->dc16x16[matrix_id] = value;
  } else if (size_id == 3) {
    s->dc32x32[matrix_id / 3] = value;
  }
}

uint8_t GetDc(const ScalingListData* s, int size_id, int matrix_id) {
  if (size_id == 2) {
    return s->dc16x16[matrix_id];
  }
  if (size_id == 3) {
    return s->dc32x32[matrix_id / 3];
  }
  return 16;
}

void FillDefaultList(ScalingListData* s, int size_id, int matrix_id) {
  const int n = (size_id == 0) ? 16 : 64;
  const uint8_t* def =
      (size_id == 0) ? kDefault4x4
                     : (matrix_id < 3 ? kDefaultIntra8x8 : kDefaultInter8x8);
  std::memcpy(CoefList(s, size_id, matrix_id), def, n);
  SetDc(s, size_id, matrix_id, 16);
}

void CopyList(ScalingListData* s, int size_id, int dst_matrix, int src_matrix) {
  const int n = (size_id == 0) ? 16 : 64;
  std::memcpy(CoefList(s, size_id, dst_matrix),
              CoefList(s, size_id, src_matrix), n);
  SetDc(s, size_id, dst_matrix, GetDc(s, size_id, src_matrix));
}

// Fills `pos` (blk_size * blk_size entries) with the raster position for each
// up-right diagonal scan index (clause 6.5.3).
void MakeDiagScan(int blk_size, uint8_t* pos) {
  int i = 0;
  int x = 0;
  int y = 0;
  bool stop = false;
  while (!stop) {
    while (y >= 0) {
      if (x < blk_size && y < blk_size) {
        pos[i++] = static_cast<uint8_t>(y * blk_size + x);
      }
      --y;
      ++x;
    }
    y = x;
    x = 0;
    if (i >= blk_size * blk_size) {
      stop = true;
    }
  }
}

}  // namespace

void SetDefaultScalingList(ScalingListData* out) {
  for (int size_id = 0; size_id < 4; ++size_id) {
    for (int matrix_id = 0; matrix_id < 6;
         matrix_id += (size_id == 3) ? 3 : 1) {
      FillDefaultList(out, size_id, matrix_id);
    }
  }
}

bool ParseScalingListData(BitReader* br, ScalingListData* out) {
  uint8_t scan4x4[16];
  uint8_t scan8x8[64];
  MakeDiagScan(4, scan4x4);
  MakeDiagScan(8, scan8x8);

  for (int size_id = 0; size_id < 4; ++size_id) {
    const uint8_t* scan = (size_id == 0) ? scan4x4 : scan8x8;
    const uint32_t coef_num = (size_id == 0) ? 16u : 64u;
    for (int matrix_id = 0; matrix_id < 6;
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
        if (pred_matrix_id_delta == 0) {
          FillDefaultList(out, size_id, matrix_id);
        } else {
          // refMatrixId = matrixId - delta * (sizeId == 3 ? 3 : 1); it must be
          // a matrix already parsed at this size.
          const uint32_t stride = (size_id == 3) ? 3u : 1u;
          if (pred_matrix_id_delta * stride >
              static_cast<uint32_t>(matrix_id)) {
            return false;
          }
          CopyList(out, size_id, matrix_id,
                   matrix_id - static_cast<int>(pred_matrix_id_delta * stride));
        }
        continue;
      }

      int32_t next_coef = 8;
      if (size_id > 1) {
        int32_t dc_minus8 = 0;
        if (!br->ReadSe(&dc_minus8)) {
          return false;
        }
        if (dc_minus8 < -7 || dc_minus8 > 247) {  // scaling_list_dc_coef range
          return false;
        }
        next_coef = dc_minus8 + 8;
        SetDc(out, size_id, matrix_id, static_cast<uint8_t>(next_coef));
      }
      uint8_t* list = CoefList(out, size_id, matrix_id);
      for (uint32_t i = 0; i < coef_num; ++i) {
        int32_t delta = 0;
        if (!br->ReadSe(&delta)) {
          return false;
        }
        if (delta < -128 || delta > 127) {  // scaling_list_delta_coef range
          return false;
        }
        // next_coef stays in 0..255: the running value is non-negative and the
        // modulo keeps it there.
        next_coef = (next_coef + delta + 256) % 256;
        list[scan[i]] = static_cast<uint8_t>(next_coef);
      }
    }
  }
  return true;
}

}  // namespace v4l2wc::h265

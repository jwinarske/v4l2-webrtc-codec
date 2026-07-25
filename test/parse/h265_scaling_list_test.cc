// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.265 scaling_list_data parser. No real stream carries
// scaling lists in the corpus, so the vectors are synthetic: the default lists,
// an all-default scaling_list_data, an explicit list that checks the diagonal
// scan placement, a copy-from-earlier list, and a 16x16 DC coefficient.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

#include "parse/bit_reader.h"
#include "parse/h265/scaling_list.h"

using v4l2wc::BitReader;
using v4l2wc::h265::ParseScalingListData;
using v4l2wc::h265::ScalingListData;
using v4l2wc::h265::SetDefaultScalingList;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

class BitWriter {
 public:
  void WriteBits(uint32_t value, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t bit = (value >> (n - 1 - i)) & 1u;
      if (bit_ == 0) bytes_.push_back(0);
      if (bit) bytes_.back() |= static_cast<uint8_t>(1u << (7 - bit_));
      bit_ = (bit_ + 1) & 7;
    }
  }
  void WriteFlag(bool v) { WriteBits(v ? 1 : 0, 1); }
  void WriteUe(uint32_t v) {
    uint32_t code = v + 1;
    uint32_t nbits = 0;
    while ((code >> nbits) != 0) ++nbits;
    WriteBits(0, nbits - 1);
    WriteBits(code, nbits);
  }
  void WriteSe(int32_t v) {
    uint32_t code = v <= 0 ? static_cast<uint32_t>(-2 * v)
                           : static_cast<uint32_t>(2 * v - 1);
    WriteUe(code);
  }
  const std::vector<uint8_t>& bytes() const { return bytes_; }

 private:
  std::vector<uint8_t> bytes_;
  uint32_t bit_ = 0;
};

// One matrix's coding: default (pred_matrix_id_delta), copy (delta > 0), or
// explicit coefficients (optional DC for 16x16 / 32x32).
struct MatrixSpec {
  bool explicit_mode = false;
  uint32_t pred_matrix_id_delta = 0;  // used when !explicit_mode
  std::optional<int32_t> dc_minus8;
  std::vector<int32_t> deltas;  // scaling_list_delta_coef sequence
};

// Emits a full scaling_list_data with `chooser` deciding each matrix.
static std::vector<uint8_t> BuildScalingListData(
    const std::function<MatrixSpec(int, int)>& chooser) {
  BitWriter w;
  for (int size_id = 0; size_id < 4; ++size_id) {
    for (int matrix_id = 0; matrix_id < 6;
         matrix_id += (size_id == 3) ? 3 : 1) {
      const MatrixSpec spec = chooser(size_id, matrix_id);
      w.WriteFlag(spec.explicit_mode);
      if (!spec.explicit_mode) {
        w.WriteUe(spec.pred_matrix_id_delta);
        continue;
      }
      if (spec.dc_minus8) w.WriteSe(*spec.dc_minus8);
      for (int32_t d : spec.deltas) w.WriteSe(d);
    }
  }
  return w.bytes();
}

// A default matrix (pred_matrix_id_delta = 0).
static MatrixSpec DefaultMatrix() { return MatrixSpec{}; }

int main() {
  // ---- The default lists. ----
  {
    ScalingListData s;
    SetDefaultScalingList(&s);
    CHECK(s.list4x4[0][0] == 16);
    CHECK(s.list4x4[5][15] == 16);
    CHECK(s.list8x8[0][0] == 16);
    CHECK(s.list8x8[0][63] == 115);  // last intra default coefficient
    CHECK(s.list8x8[3][63] == 91);   // last inter default coefficient
    CHECK(s.list16x16[2][63] == 115);
    CHECK(s.list32x32[0][63] == 115);  // matrixId 0 (intra)
    CHECK(s.list32x32[1][63] == 91);   // matrixId 3 (inter)
    CHECK(s.dc16x16[0] == 16);
    CHECK(s.dc32x32[0] == 16);
  }

  // ---- An all-default scaling_list_data parses to the default lists. ----
  {
    const std::vector<uint8_t> rbsp =
        BuildScalingListData([](int, int) { return DefaultMatrix(); });
    ScalingListData parsed;
    BitReader br(rbsp.data(), rbsp.size());
    CHECK(ParseScalingListData(&br, &parsed));
    ScalingListData def;
    SetDefaultScalingList(&def);
    CHECK(std::memcmp(&parsed, &def, sizeof(ScalingListData)) == 0);
  }

  // ---- Explicit 4x4 matrix 0: check the diagonal-scan placement. The coded
  // values run 100..115; the up-right diagonal scan places index 0 at raster 0,
  // index 1 at raster 4, index 2 at raster 1. ----
  {
    const std::vector<uint8_t> rbsp =
        BuildScalingListData([](int size_id, int matrix_id) -> MatrixSpec {
          if (size_id == 0 && matrix_id == 0) {
            MatrixSpec m;
            m.explicit_mode = true;
            m.deltas.push_back(92);  // nextCoef = (8 + 92 + 256) % 256 = 100
            for (int i = 0; i < 15; ++i) m.deltas.push_back(1);  // 101..115
            return m;
          }
          return DefaultMatrix();
        });
    ScalingListData parsed;
    BitReader br(rbsp.data(), rbsp.size());
    CHECK(ParseScalingListData(&br, &parsed));
    CHECK(parsed.list4x4[0][0] == 100);  // scan index 0
    CHECK(parsed.list4x4[0][4] == 101);  // scan index 1 -> (x=0, y=1)
    CHECK(parsed.list4x4[0][1] == 102);  // scan index 2 -> (x=1, y=0)
  }

  // ---- Copy-from-earlier: 4x4 matrix 0 explicit (all 20), matrix 1 copies it
  // (pred_matrix_id_delta = 1). ----
  {
    const std::vector<uint8_t> rbsp =
        BuildScalingListData([](int size_id, int matrix_id) -> MatrixSpec {
          if (size_id == 0 && matrix_id == 0) {
            MatrixSpec m;
            m.explicit_mode = true;
            m.deltas.push_back(12);  // nextCoef = 20
            for (int i = 0; i < 15; ++i) m.deltas.push_back(0);
            return m;
          }
          if (size_id == 0 && matrix_id == 1) {
            MatrixSpec m;
            m.pred_matrix_id_delta = 1;  // copy matrix 0
            return m;
          }
          return DefaultMatrix();
        });
    ScalingListData parsed;
    BitReader br(rbsp.data(), rbsp.size());
    CHECK(ParseScalingListData(&br, &parsed));
    CHECK(parsed.list4x4[0][0] == 20);
    CHECK(std::memcmp(parsed.list4x4[0], parsed.list4x4[1], 16) == 0);
  }

  // ---- 16x16 DC coefficient: scaling_list_dc_coef_minus8 = 5 -> DC 13. ----
  {
    const std::vector<uint8_t> rbsp =
        BuildScalingListData([](int size_id, int matrix_id) -> MatrixSpec {
          if (size_id == 2 && matrix_id == 0) {
            MatrixSpec m;
            m.explicit_mode = true;
            m.dc_minus8 = 5;         // DC = 13
            m.deltas.assign(64, 0);  // nextCoef stays at DC value
            return m;
          }
          return DefaultMatrix();
        });
    ScalingListData parsed;
    BitReader br(rbsp.data(), rbsp.size());
    CHECK(ParseScalingListData(&br, &parsed));
    CHECK(parsed.dc16x16[0] == 13);
    CHECK(parsed.list16x16[0][0] == 13);  // first coef seeded from DC
  }

  if (g_failures == 0) {
    std::printf("H265_SCALING_LIST_TEST_OK\n");
    return 0;
  }
  std::printf("H265_SCALING_LIST_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

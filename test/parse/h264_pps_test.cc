// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.264 PPS parser. Runs on the host (no webrtc / V4L2).

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h264/nal.h"
#include "parse/h264/pps.h"

using v4l2wc::h264::NalUnitType;
using v4l2wc::h264::ParseAnnexB;
using v4l2wc::h264::ParsePps;
using v4l2wc::h264::Pps;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

int main() {
  // Minimal CAVLC PPS, hand-encoded and bit-verified: pps_id=0, sps_id=0,
  // entropy_coding_mode=0, one slice group, ref-idx defaults 0, qp deltas 0,
  // no extension, then rbsp trailing bits.
  const std::vector<uint8_t> rbsp = {0xce, 0x38, 0x80};
  {
    Pps pps;
    CHECK(ParsePps(rbsp.data(), rbsp.size(), &pps));
    CHECK(pps.pic_parameter_set_id == 0);
    CHECK(pps.seq_parameter_set_id == 0);
    CHECK(!pps.entropy_coding_mode_flag);
    CHECK(pps.num_ref_idx_l0_default_active_minus1 == 0);
    CHECK(pps.num_ref_idx_l1_default_active_minus1 == 0);
    CHECK(!pps.transform_8x8_mode_flag);  // no extension present
  }

  // Integration: extract the PPS NAL (0x68 = type 8) from an Annex-B stream.
  {
    std::vector<uint8_t> stream = {0x00, 0x00, 0x00, 0x01, 0x68};
    stream.insert(stream.end(), rbsp.begin(), rbsp.end());
    auto nals = ParseAnnexB(stream.data(), stream.size());
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kPps);
      Pps pps;
      CHECK(ParsePps(nals[0].rbsp.data(), nals[0].rbsp.size(), &pps));
      CHECK(pps.pic_parameter_set_id == 0);
    }
  }

  // Malformed / truncated input must fail cleanly, never crash.
  {
    Pps pps;
    CHECK(!ParsePps(nullptr, 0, &pps));
    CHECK(!ParsePps(rbsp.data(), 0, &pps));
    const std::vector<uint8_t> zeros(8, 0x00);
    CHECK(!ParsePps(zeros.data(), zeros.size(), &pps));  // ue never terminates
  }

  if (g_failures == 0) {
    std::printf("H264_PPS_TEST_OK\n");
    return 0;
  }
  std::printf("H264_PPS_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

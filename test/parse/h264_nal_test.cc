// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.264 Annex-B NAL parser. No framework: a failing check
// prints and sets the exit code. Runs on the host (no webrtc / V4L2).

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h264/nal.h"

using v4l2wc::h264::Nal;
using v4l2wc::h264::NalUnitType;
using v4l2wc::h264::ParseAnnexB;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

static std::vector<Nal> Parse(const std::vector<uint8_t>& v) {
  return ParseAnnexB(v.data(), v.size());
}

int main() {
  // 4-byte start code + SPS (0x67: forbidden 0, ref_idc 3, type 7) whose
  // payload carries an emulation-prevention byte (00 00 03 01 -> 00 00 01).
  {
    auto nals = Parse({0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x1f, 0x00,
                       0x00, 0x03, 0x01});
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kSps);
      CHECK(nals[0].nal_ref_idc == 3);
      const std::vector<uint8_t> want = {0x42, 0x00, 0x1f, 0x00, 0x00, 0x01};
      CHECK(nals[0].rbsp == want);  // 0x03 stripped
    }
  }

  // Two NALs: SPS (4-byte start code) then PPS (3-byte start code, 0x68 type
  // 8).
  {
    auto nals = Parse({0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x01,  //
                       0x00, 0x00, 0x01, 0x68, 0xce, 0x3c, 0x80});
    CHECK(nals.size() == 2);
    if (nals.size() == 2) {
      CHECK(nals[0].type == NalUnitType::kSps);
      CHECK(nals[1].type == NalUnitType::kPps);
      CHECK(nals[1].nal_ref_idc == 3);  // 0x68 >> 5 == 3
    }
  }

  // forbidden_zero_bit set (0xE7) -> the NAL is rejected.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0xe7, 0x11, 0x22});
    CHECK(nals.empty());
  }

  // No start code -> no NALs, no crash.
  CHECK(Parse({0x00, 0x00}).empty());
  CHECK(Parse({}).empty());
  CHECK(Parse({0xde, 0xad, 0xbe, 0xef}).empty());

  // Start code with only a header byte (empty payload).
  {
    auto nals =
        Parse({0x00, 0x00, 0x01, 0x09, 0x10});  // AUD, then 0x10 payload
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kAccessUnitDelimiter);
    }
  }

  // Emulation-prevention byte at the very end must not read past the buffer.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x06, 0x00, 0x00, 0x03});  // SEI
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kSei);
      const std::vector<uint8_t> want = {0x00, 0x00};  // trailing 0x03 dropped
      CHECK(nals[0].rbsp == want);
    }
  }

  if (g_failures == 0) {
    std::printf("H264_NAL_TEST_OK\n");
    return 0;
  }
  std::printf("H264_NAL_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.265 Annex-B NAL parser. No framework: a failing check
// prints and sets the exit code. Runs on the host (no webrtc / V4L2).

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h265/nal.h"

using v4l2wc::h265::IsIdr;
using v4l2wc::h265::IsIrap;
using v4l2wc::h265::IsVcl;
using v4l2wc::h265::Nal;
using v4l2wc::h265::NalUnitType;
using v4l2wc::h265::ParseAnnexB;
using v4l2wc::h265::RbspToRawBitOffset;

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
  // VPS (type 32, header 40 01), SPS (type 33, header 42 01), PPS (type 34,
  // header 44 01). A 4-byte start code then two 3-byte ones.
  {
    auto nals = Parse({0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0xaa,  // VPS
                       0x00, 0x00, 0x01, 0x42, 0x01, 0xbb,        // SPS
                       0x00, 0x00, 0x01, 0x44, 0x01, 0xcc});      // PPS
    CHECK(nals.size() == 3);
    if (nals.size() == 3) {
      CHECK(nals[0].type == NalUnitType::kVpsNut);
      CHECK(nals[1].type == NalUnitType::kSpsNut);
      CHECK(nals[2].type == NalUnitType::kPpsNut);
      CHECK(nals[1].nuh_layer_id == 0);
      CHECK(nals[1].nuh_temporal_id_plus1 == 1);
      const std::vector<uint8_t> want = {0xbb};  // payload after the 2-byte hdr
      CHECK(nals[1].rbsp == want);
    }
  }

  // The 6-bit type is read from the right bits: 33 shifted into the header must
  // come back as 33, not the whole first byte or a 5-bit value.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x42, 0x01, 0x00});  // SPS
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(static_cast<uint8_t>(nals[0].type) == 33);
    }
  }

  // nuh_layer_id straddles the two header bytes: its top bit is the low bit of
  // byte0 and its low five bits are the top of byte1. Layer 1 with type 33 is
  // header 0x42 0x09.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x42, 0x09, 0x00});
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].nuh_layer_id == 1);
      CHECK(nals[0].nuh_temporal_id_plus1 == 1);
    }
  }

  // A non-zero temporal id: byte1 low three bits carry temporal_id_plus1.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x26, 0x03, 0x00});  // IDR, tid+1 = 3
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kIdrWRadl);
      CHECK(nals[0].nuh_temporal_id_plus1 == 3);
    }
  }

  // An emulation-prevention byte in the payload: 00 00 03 01 -> 00 00 01.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x42, 0x01, 0x00, 0x00, 0x03, 0x01});
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      const std::vector<uint8_t> want = {0x00, 0x00, 0x01};  // 0x03 stripped
      CHECK(nals[0].rbsp == want);
    }
  }

  // forbidden_zero_bit set (byte0 top bit) -> the NAL is rejected.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0xc2, 0x01, 0x00});
    CHECK(nals.empty());
  }

  // nuh_temporal_id_plus1 == 0 is reserved and never valid -> rejected.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x42, 0x00, 0x00});
    CHECK(nals.empty());
  }

  // A unit shorter than the two-byte header is malformed -> rejected. (The
  // trailing zero is trimmed, leaving a single header byte.)
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x42, 0x00});
    CHECK(nals.empty());
  }

  // No start code, and empty input: no NALs, no crash.
  CHECK(Parse({0x00, 0x00}).empty());
  CHECK(Parse({}).empty());
  CHECK(Parse({0xde, 0xad, 0xbe, 0xef}).empty());

  // An emulation byte at the very end must not read past the buffer.
  {
    auto nals = Parse({0x00, 0x00, 0x01, 0x4e, 0x01, 0x00, 0x00, 0x03});
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kPrefixSeiNut);  // type 39
      const std::vector<uint8_t> want = {0x00, 0x00};  // trailing 0x03 dropped
      CHECK(nals[0].rbsp == want);
    }
  }

  // Classification helpers.
  CHECK(IsVcl(NalUnitType::kTrailR));
  CHECK(IsVcl(NalUnitType::kIdrWRadl));
  CHECK(!IsVcl(NalUnitType::kSpsNut));
  CHECK(IsIrap(NalUnitType::kIdrNLp));
  CHECK(IsIrap(NalUnitType::kCraNut));
  CHECK(!IsIrap(NalUnitType::kTrailR));
  CHECK(IsIdr(NalUnitType::kIdrWRadl));
  CHECK(IsIdr(NalUnitType::kIdrNLp));
  CHECK(!IsIdr(NalUnitType::kCraNut));

  // RbspToRawBitOffset accounts for the two header bytes and any emulation
  // byte before the offset. Payload 00 00 03 01 55 -> RBSP 00 00 01 55, so the
  // 0x03 sits strictly before RBSP byte index 3 (0x55).
  {
    auto nals =
        Parse({0x00, 0x00, 0x01, 0x42, 0x01, 0x00, 0x00, 0x03, 0x01, 0x55});
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      const std::vector<uint8_t> want = {0x00, 0x00, 0x01, 0x55};
      CHECK(nals[0].rbsp == want);
      uint32_t raw = 0;
      // Offset 0 in the RBSP is right after the 16-bit header.
      CHECK(RbspToRawBitOffset(nals[0], 0, &raw));
      CHECK(raw == 16);
      // RBSP byte index 3 sits after the dropped 0x03, so its raw position is
      // shifted one byte (8 bits) past the header + RBSP offset.
      CHECK(RbspToRawBitOffset(nals[0], 3 * 8, &raw));
      CHECK(raw == 16 + 3 * 8 + 8);
    }
  }

  if (g_failures == 0) {
    std::printf("H265_NAL_TEST_OK\n");
    return 0;
  }
  std::printf("H265_NAL_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

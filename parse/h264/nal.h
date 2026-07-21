// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.264 Annex-B NAL unit parsing.
//
// The bitstream is attacker-controlled and reachable after the DTLS handshake,
// so every read here is bounds-checked as a stated invariant (not an assert):
// malformed input yields fewer/zero NAL units, never a read past the buffer.
// This is a pure-logic layer with no webrtc or V4L2 dependency, so it is
// unit-tested and fuzzed on the host.
#ifndef V4L2WC_PARSE_H264_NAL_H_
#define V4L2WC_PARSE_H264_NAL_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace v4l2wc::h264 {

// H.264 NAL unit types (Table 7-1); only the ones this decoder cares about are
// named, the rest pass through as their numeric value.
enum class NalUnitType : uint8_t {
  kUnspecified = 0,
  kSliceNonIdr = 1,
  kSlicePartitionA = 2,
  kSlicePartitionB = 3,
  kSlicePartitionC = 4,
  kSliceIdr = 5,
  kSei = 6,
  kSps = 7,
  kPps = 8,
  kAccessUnitDelimiter = 9,
  kEndOfSequence = 10,
  kEndOfStream = 11,
};

struct Nal {
  NalUnitType type = NalUnitType::kUnspecified;
  uint8_t nal_ref_idc = 0;  // 0..3
  // RBSP: the NAL payload (after the 1-byte header) with emulation-prevention
  // bytes removed. Owned; safe to read start..start+size().
  std::vector<uint8_t> rbsp;
};

// Splits an Annex-B byte stream into NAL units. Start codes are 0x000001 or
// 0x00000001; the leading bytes before the first start code are ignored. Each
// NAL's header is validated (forbidden_zero_bit must be 0) and its payload has
// emulation-prevention bytes (0x000003 -> 0x0000) stripped. Never reads past
// `data + size`; `data` may be null only when `size` is 0.
std::vector<Nal> ParseAnnexB(const uint8_t* data, size_t size);

}  // namespace v4l2wc::h264

#endif  // V4L2WC_PARSE_H264_NAL_H_

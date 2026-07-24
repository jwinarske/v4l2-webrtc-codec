// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.265 / HEVC Annex-B NAL unit parsing.
//
// The bitstream is attacker-controlled and reachable after the DTLS handshake,
// so every read here is bounds-checked as a stated invariant (not an assert):
// malformed input yields fewer/zero NAL units, never a read past the buffer.
// This is a pure-logic layer with no webrtc or V4L2 dependency, so it is
// unit-tested and fuzzed on the host.
//
// HEVC differs from H.264 only in the NAL header, which is two bytes rather
// than one, and in the type table. Start codes and emulation prevention are
// the same.
#ifndef V4L2WC_PARSE_H265_NAL_H_
#define V4L2WC_PARSE_H265_NAL_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace v4l2wc::h265 {

// HEVC NAL unit types (Table 7-1). Types 0..31 are VCL (slice) units; 32 and
// up are non-VCL. Only the ones a decoder distinguishes are named; the rest
// pass through as their numeric value.
enum class NalUnitType : uint8_t {
  kTrailN = 0,
  kTrailR = 1,
  kTsaN = 2,
  kTsaR = 3,
  kStsaN = 4,
  kStsaR = 5,
  kRadlN = 6,
  kRadlR = 7,
  kRaslN = 8,
  kRaslR = 9,
  kBlaWLp = 16,
  kBlaWRadl = 17,
  kBlaNLp = 18,
  kIdrWRadl = 19,
  kIdrNLp = 20,
  kCraNut = 21,
  kVpsNut = 32,
  kSpsNut = 33,
  kPpsNut = 34,
  kAudNut = 35,
  kEosNut = 36,
  kEobNut = 37,
  kFdNut = 38,
  kPrefixSeiNut = 39,
  kSuffixSeiNut = 40,
};

// A slice unit carries coded picture data; everything else is a parameter set,
// delimiter, or metadata.
[[nodiscard]] bool IsVcl(NalUnitType type);
// An intra random-access point: BLA, IDR, or CRA. The start of a decodable
// segment.
[[nodiscard]] bool IsIrap(NalUnitType type);
// An IDR specifically, which resets the decoded-picture buffer.
[[nodiscard]] bool IsIdr(NalUnitType type);
// A BLA (broken-link access) picture. Like an IDR it starts a new coded video
// sequence, so NoRaslOutputFlag is 1 and the POC MSB resets.
[[nodiscard]] bool IsBla(NalUnitType type);

struct Nal {
  Nal();
  ~Nal();
  Nal(const Nal&);
  Nal& operator=(const Nal&);
  // Declared alongside the destructor, which would otherwise suppress them:
  // NALs are moved into the parse result, and deep-copying two buffers per NAL
  // instead would be a silent regression.
  Nal(Nal&&) noexcept;
  Nal& operator=(Nal&&) noexcept;

  NalUnitType type = NalUnitType::kTrailN;
  uint8_t nuh_layer_id = 0;           // 0..63
  uint8_t nuh_temporal_id_plus1 = 0;  // 1..7; temporal_id is this minus one
  // RBSP: the NAL payload (after the 2-byte header) with emulation-prevention
  // bytes removed. Owned; safe to read start..start+size().
  std::vector<uint8_t> rbsp;
  // The NAL exactly as it appeared in the stream: the two header bytes plus
  // payload, start code excluded, emulation-prevention bytes intact. Hardware
  // decoders submit this buffer and address into it in raw bit space.
  std::vector<uint8_t> raw;
};

// Converts a bit offset within `nal.rbsp` to the corresponding bit offset
// within `nal.raw`, accounting for the 2-byte NAL header and any
// emulation-prevention bytes removed before that point. This is what a VA-API
// or stateless decoder needs for slice_data addressing. Returns false if the
// result would fall outside `nal.raw`.
bool RbspToRawBitOffset(const Nal& nal, uint32_t rbsp_bit_offset,
                        uint32_t* raw_bit_offset);

// Splits an Annex-B byte stream into NAL units. Start codes are 0x000001 or
// 0x00000001; the leading bytes before the first start code are ignored. Each
// NAL's two-byte header is validated (forbidden_zero_bit must be 0, and the
// unit must be at least two bytes) and its payload has emulation-prevention
// bytes (0x000003 -> 0x0000) stripped. Never reads past `data + size`; `data`
// may be null only when `size` is 0.
std::vector<Nal> ParseAnnexB(const uint8_t* data, size_t size);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_NAL_H_

// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/nal.h"

namespace v4l2wc::h265 {

Nal::Nal() = default;
Nal::~Nal() = default;
Nal::Nal(const Nal&) = default;
Nal& Nal::operator=(const Nal&) = default;
Nal::Nal(Nal&&) noexcept = default;
Nal& Nal::operator=(Nal&&) noexcept = default;

bool IsVcl(NalUnitType type) { return static_cast<uint8_t>(type) <= 31; }

bool IsIrap(NalUnitType type) {
  const uint8_t t = static_cast<uint8_t>(type);
  return t >= 16 && t <= 23;  // BLA_W_LP .. RSV_IRAP_VCL23
}

bool IsIdr(NalUnitType type) {
  return type == NalUnitType::kIdrWRadl || type == NalUnitType::kIdrNLp;
}

bool IsBla(NalUnitType type) {
  const uint8_t t = static_cast<uint8_t>(type);
  return t >= 16 && t <= 18;  // BLA_W_LP .. BLA_N_LP
}

namespace {

// Removes emulation-prevention bytes: 0x03 in a `0x00 0x00 0x03` sequence
// where the byte after the 0x03 is <= 0x03 (or the 0x03 ends the buffer).
// Bounds-safe: p[i + 1] is read only when i + 1 < n. Identical to H.264.
std::vector<uint8_t> StripEmulationPrevention(const uint8_t* p, size_t n) {
  std::vector<uint8_t> out;
  out.reserve(n);
  size_t zeros = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t b = p[i];
    if (zeros >= 2 && b == 0x03 && (i + 1 >= n || p[i + 1] <= 0x03)) {
      zeros = 0;  // drop the emulation-prevention byte
      continue;
    }
    out.push_back(b);
    zeros = (b == 0) ? zeros + 1 : 0;
  }
  return out;
}

// Finds the next 3-byte start code (0x00 0x00 0x01) at or after `from`.
// Returns its offset in `pos`, or false if none. A 4-byte start code matches
// on its trailing three bytes; the extra leading zero is trimmed as a
// preceding NAL's trailing byte.
bool FindStartCode(const uint8_t* data, size_t size, size_t from, size_t* pos) {
  for (size_t j = from; j + 3 <= size; ++j) {
    if (data[j] == 0 && data[j + 1] == 0 && data[j + 2] == 1) {
      *pos = j;
      return true;
    }
  }
  return false;
}

}  // namespace

std::vector<Nal> ParseAnnexB(const uint8_t* data, size_t size) {
  std::vector<Nal> nals;
  if (data == nullptr || size == 0) {
    return nals;
  }

  size_t start_code;
  if (!FindStartCode(data, size, 0, &start_code)) {
    return nals;
  }
  size_t nal_start = start_code + 3;  // first byte after the start code

  while (nal_start < size) {
    size_t nal_end;
    size_t next_nal_start;
    const bool has_next = FindStartCode(data, size, nal_start, &nal_end);
    if (has_next) {
      next_nal_start = nal_end + 3;
    } else {
      nal_end = size;
      next_nal_start = size;
    }

    // Trailing zero bytes belong to start-code alignment, not the NAL.
    while (nal_end > nal_start && data[nal_end - 1] == 0) {
      --nal_end;
    }

    // The HEVC NAL header is two bytes, so a shorter unit is malformed.
    if (nal_end - nal_start >= 2) {
      const uint8_t byte0 = data[nal_start];
      const uint8_t byte1 = data[nal_start + 1];
      const uint8_t temporal_id_plus1 = byte1 & 0x07;
      // forbidden_zero_bit must be 0, and nuh_temporal_id_plus1 must be
      // non-zero (temporal_id_plus1 == 0 is reserved and never valid).
      if ((byte0 & 0x80) == 0 && temporal_id_plus1 != 0) {
        Nal nal;
        nal.type = static_cast<NalUnitType>((byte0 >> 1) & 0x3f);
        nal.nuh_layer_id =
            static_cast<uint8_t>(((byte0 & 0x01) << 5) | ((byte1 >> 3) & 0x1f));
        nal.nuh_temporal_id_plus1 = temporal_id_plus1;
        nal.rbsp = StripEmulationPrevention(data + nal_start + 2,
                                            nal_end - (nal_start + 2));
        nal.raw.assign(data + nal_start, data + nal_end);
        nals.push_back(std::move(nal));
      }
    }

    if (!has_next) {
      break;
    }
    nal_start = next_nal_start;
  }
  return nals;
}

bool RbspToRawBitOffset(const Nal& nal, uint32_t rbsp_bit_offset,
                        uint32_t* raw_bit_offset) {
  if (raw_bit_offset == nullptr || nal.raw.size() < 2) {
    return false;
  }
  // Count the emulation-prevention bytes dropped from the payload before the
  // RBSP byte the offset falls in; each shifts the raw position by one byte.
  const size_t rbsp_byte = rbsp_bit_offset >> 3;
  if (rbsp_byte > nal.rbsp.size()) {
    return false;
  }
  const uint8_t* payload = nal.raw.data() + 2;  // skip the two header bytes
  const size_t payload_size = nal.raw.size() - 2;
  size_t epb = 0;
  size_t zeros = 0;
  for (size_t i = 0, kept = 0; i < payload_size && kept < rbsp_byte; ++i) {
    const uint8_t b = payload[i];
    if (zeros >= 2 && b == 0x03 &&
        (i + 1 >= payload_size || payload[i + 1] <= 0x03)) {
      ++epb;
      zeros = 0;
      continue;
    }
    ++kept;
    zeros = (b == 0) ? zeros + 1 : 0;
  }
  // 16 bits for the two NAL header bytes, plus the payload offset shifted by
  // the emulation bytes that precede it.
  const uint64_t raw_bits =
      16ull + static_cast<uint64_t>(rbsp_bit_offset) + 8ull * epb;
  if (raw_bits > static_cast<uint64_t>(nal.raw.size()) * 8ull) {
    return false;
  }
  *raw_bit_offset = static_cast<uint32_t>(raw_bits);
  return true;
}

}  // namespace v4l2wc::h265

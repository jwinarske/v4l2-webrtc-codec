// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h264/nal.h"

namespace v4l2wc::h264 {
namespace {

// Removes emulation-prevention bytes: 0x03 in a `0x00 0x00 0x03` sequence
// where the byte after the 0x03 is <= 0x03 (or the 0x03 ends the buffer).
// Bounds-safe: p[i + 1] is read only when i + 1 < n.
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
// Returns its offset in `pos`, or false if none. A 4-byte start code
// (0x00 0x00 0x00 0x01) matches on its trailing three bytes; the extra leading
// zero is trimmed as a preceding NAL's trailing byte.
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
    bool has_next = FindStartCode(data, size, nal_start, &nal_end);
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

    if (nal_end > nal_start) {
      const uint8_t header = data[nal_start];
      if ((header & 0x80) == 0) {  // forbidden_zero_bit must be 0
        Nal nal;
        nal.nal_ref_idc = (header >> 5) & 0x03;
        nal.type = static_cast<NalUnitType>(header & 0x1f);
        nal.rbsp = StripEmulationPrevention(data + nal_start + 1,
                                            nal_end - (nal_start + 1));
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
  if (raw_bit_offset == nullptr || nal.raw.empty()) {
    return false;
  }
  // Count the emulation-prevention bytes dropped from the payload before the
  // RBSP byte the offset falls in; each shifts the raw position by one byte.
  const size_t rbsp_byte = rbsp_bit_offset >> 3;
  if (rbsp_byte > nal.rbsp.size()) {
    return false;
  }
  const uint8_t* payload = nal.raw.data() + 1;  // skip the NAL header byte
  const size_t payload_size = nal.raw.size() - 1;
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
  // 8 bits for the NAL header byte, plus the payload offset shifted by the
  // emulation bytes that precede it.
  const uint64_t raw_bits =
      8ull + static_cast<uint64_t>(rbsp_bit_offset) + 8ull * epb;
  if (raw_bits > static_cast<uint64_t>(nal.raw.size()) * 8ull) {
    return false;
  }
  *raw_bit_offset = static_cast<uint32_t>(raw_bits);
  return true;
}

}  // namespace v4l2wc::h264

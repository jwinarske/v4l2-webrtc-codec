// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Bounds-checked big-endian bit reader with Exp-Golomb decoding, for RBSP
// parsing. Every read is checked against the buffer end: running out of bits
// returns false rather than reading past the buffer. Attacker-controlled input,
// so this is a stated invariant, not an assert.
#ifndef V4L2WC_PARSE_BIT_READER_H_
#define V4L2WC_PARSE_BIT_READER_H_

#include <cstddef>
#include <cstdint>

namespace v4l2wc {

class BitReader {
 public:
  BitReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

  // Reads `n` (0..32) bits MSB-first into *out. False if fewer than n remain.
  bool ReadBits(uint32_t n, uint32_t* out) {
    if (n > 32) {
      return false;
    }
    uint32_t value = 0;
    for (uint32_t i = 0; i < n; ++i) {
      if (bit_pos_ >= size_ * 8) {
        return false;
      }
      const uint32_t byte = data_[bit_pos_ >> 3];
      const uint32_t bit = (byte >> (7 - (bit_pos_ & 7))) & 1u;
      value = (value << 1) | bit;
      ++bit_pos_;
    }
    *out = value;
    return true;
  }

  bool ReadFlag(bool* out) {
    uint32_t bit = 0;
    if (!ReadBits(1, &bit)) {
      return false;
    }
    *out = bit != 0;
    return true;
  }

  // Unsigned Exp-Golomb ue(v). The leading-zero run is capped at 31 so the
  // result cannot overflow uint32 and a malformed all-zeros stream terminates.
  bool ReadUe(uint32_t* out) {
    uint32_t zeros = 0;
    for (;;) {
      uint32_t bit = 0;
      if (!ReadBits(1, &bit)) {
        return false;
      }
      if (bit != 0) {
        break;
      }
      if (++zeros > 31) {
        return false;
      }
    }
    uint32_t suffix = 0;
    if (zeros > 0 && !ReadBits(zeros, &suffix)) {
      return false;
    }
    *out = ((zeros >= 32) ? 0u : (1u << zeros)) - 1u + suffix;
    return true;
  }

  // Signed Exp-Golomb se(v): 0,1,-1,2,-2,...
  bool ReadSe(int32_t* out) {
    uint32_t code = 0;
    if (!ReadUe(&code)) {
      return false;
    }
    const uint32_t magnitude = (code + 1u) >> 1;
    *out = (code & 1u) ? static_cast<int32_t>(magnitude)
                       : -static_cast<int32_t>(magnitude);
    return true;
  }

  // Skips `n` bits. False if fewer remain.
  bool SkipBits(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
      uint32_t bit = 0;
      if (!ReadBits(1, &bit)) {
        return false;
      }
    }
    return true;
  }

 private:
  const uint8_t* data_;
  size_t size_;
  size_t bit_pos_ = 0;
};

}  // namespace v4l2wc

#endif  // V4L2WC_PARSE_BIT_READER_H_

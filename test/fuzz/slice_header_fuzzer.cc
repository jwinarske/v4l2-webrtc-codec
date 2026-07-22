// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over the slice-header parser. The first byte selects an
// SPS/PPS context variant so different code paths (weighted prediction, POC
// types, ref-list modification, MMCO) are exercised; the rest is the RBSP.
// Build:
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined -I. \
//       parse/h264/slice_header.cc test/fuzz/slice_header_fuzzer.cc -o
//       sh_fuzzer

#include <cstddef>
#include <cstdint>

#include "parse/h264/slice_header.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  if (size < 2) {
    return 0;
  }
  const uint8_t sel = data[0];
  v4l2wc::h264::SliceContext ctx;
  ctx.weighted_pred_flag = (sel & 0x01) != 0;
  ctx.weighted_bipred_idc = (sel >> 1) & 0x03;
  ctx.frame_mbs_only_flag = (sel >> 3) & 0x01;
  ctx.pic_order_cnt_type = (sel >> 4) % 3;
  ctx.redundant_pic_cnt_present_flag = (sel >> 6) & 0x01;
  ctx.bottom_field_pic_order_in_frame_present_flag = (sel >> 7) & 0x01;
  ctx.chroma_array_type = (sel & 0x08) ? 0 : 1;

  const bool idr = (data[1] & 0x01) != 0;
  v4l2wc::h264::SliceHeader sh;
  if (v4l2wc::h264::ParseSliceHeader(data + 2, size - 2, /*nal_ref_idc=*/3, idr,
                                     ctx, &sh)) {
    // On success the reported slice-data offset must lie within the RBSP it
    // was parsed from; a hardware decoder hands this to the device.
    if (sh.slice_data_bit_offset_rbsp > (size - 2) * 8) {
      __builtin_trap();
    }
  }
  return 0;
}

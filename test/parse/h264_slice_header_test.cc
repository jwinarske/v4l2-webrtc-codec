// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.264 slice-header parser. Runs on the host.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h264/slice_header.h"

using v4l2wc::h264::ParseSliceHeader;
using v4l2wc::h264::SliceContext;
using v4l2wc::h264::SliceHeader;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

int main() {
  SliceContext ctx;  // defaults: log2_max_frame_num=4, poc_type=0,
                     // log2_max_pic_order_cnt_lsb=4, frame_mbs_only, 4:2:0.

  // IDR I-slice header (nal_ref_idc=3), hand-encoded and bit-verified:
  // first_mb=0, slice_type=7 (I), pps_id=0, frame_num=0 (4 bits), idr_pic_id=0,
  // pic_order_cnt_lsb=0 (4 bits), dec_ref: no_output=0, long_term_ref=0,
  // then slice_qp_delta=0 (a single '1' bit at position 20). Slice data
  // therefore begins at bit 21.
  {
    const std::vector<uint8_t> rbsp = {0x88, 0x84, 0x08};
    SliceHeader sh;
    CHECK(ParseSliceHeader(rbsp.data(), rbsp.size(), 3, true, ctx, &sh));
    CHECK(sh.first_mb_in_slice == 0);
    CHECK(sh.slice_type == 7);
    CHECK(sh.pic_parameter_set_id == 0);
    CHECK(sh.frame_num == 0);
    CHECK(sh.idr);
    CHECK(sh.idr_pic_id == 0);
    CHECK(sh.pic_order_cnt_lsb == 0);
    CHECK(sh.ref_pic_list_mod_l0.empty());  // I slice
    CHECK(sh.mmco.empty());
    CHECK(!sh.no_output_of_prior_pics_flag);
    CHECK(!sh.long_term_reference_flag);
    CHECK(sh.slice_qp_delta == 0);
    CHECK(sh.slice_data_bit_offset_rbsp == 21);
  }

  // Malformed / truncated input must fail cleanly, never crash.
  {
    SliceHeader sh;
    CHECK(!ParseSliceHeader(nullptr, 0, 3, true, ctx, &sh));
    const std::vector<uint8_t> rbsp = {0x88, 0x84, 0x00};
    CHECK(!ParseSliceHeader(rbsp.data(), 0, 3, true, ctx, &sh));
    const std::vector<uint8_t> zeros(8, 0x00);
    CHECK(!ParseSliceHeader(zeros.data(), zeros.size(), 3, false, ctx, &sh));
  }

  if (g_failures == 0) {
    std::printf("H264_SLICE_HEADER_TEST_OK\n");
    return 0;
  }
  std::printf("H264_SLICE_HEADER_TEST_FAIL (%d)\n", g_failures);
  return 1;
}

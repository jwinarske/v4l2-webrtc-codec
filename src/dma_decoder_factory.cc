// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/dma_decoder_factory.h"

#include <linux/videodev2.h>

#include <algorithm>
#include <cstddef>

#include "src/log.h"
#include "src/v4l2_m2m_decoder.h"
#include "src/v4l2_stateless_h265_decoder.h"
#if V4L2WC_HAVE_VAAPI
#include "src/vaapi_h264_decoder.h"
#include "src/vaapi_h265_decoder.h"
#endif

namespace v4l2wc {

std::unique_ptr<IDmaDecoder> CreateDmaDecoder(const DmaDecoderConfig& config) {
  const char* device = (config.v4l2_device != nullptr && config.v4l2_device[0])
                           ? config.v4l2_device
                           : "/dev/video10";

  // OUTPUT holds one coded access unit. The luma size is not a safe bound -- a
  // high-bitrate keyframe exceeds it -- so it is floored at 1 MiB, which
  // comfortably holds an SD/HD frame.
  const std::size_t output_buffer_size = std::max<std::size_t>(
      static_cast<std::size_t>(config.coded_width) * config.coded_height,
      std::size_t{1} << 20);

  // The V4L2 M2M stateful engine first: the embedded path.
  if (auto v4l2 = V4l2M2mDecoder::Create(
          device, config.codec_fourcc, V4L2_PIX_FMT_NV12, config.coded_width,
          config.coded_height, /*output_buffer_count=*/4,
          /*capture_buffer_count=*/16, output_buffer_size)) {
    V4L2WC_LOG(V4L2WC_INFO) << "v4l2wc: V4L2 M2M decoder on " << device << " "
                            << config.coded_width << "x" << config.coded_height;
    return v4l2;
  }

  // The V4L2 stateless HEVC engine (e.g. rpi-hevc-dec on the Pi): the embedded
  // HEVC path, which the M2M stateful codec above does not carry.
  if (config.codec_fourcc == V4L2_PIX_FMT_HEVC) {
    const char* hevc_node =
        (config.v4l2_hevc_device != nullptr && config.v4l2_hevc_device[0])
            ? config.v4l2_hevc_device
            : "/dev/video19";
    // Pool must cover the max DPB (reference + reorder pictures the decoder
    // keeps live) PLUS the frames in flight to the compositor -- on the RPi
    // stateless decoder the CAPTURE buffers double as DPB slots, so every
    // buffer held on a plane removes a DPB slot. 16 left too few once the
    // KMS-plane compositor held ~8, starving the DPB ("Missing inuse DPB ent")
    // and stalling decode a few seconds in. 28 matches the VAAPI path's
    // headroom below.
    if (auto stateless =
            V4l2StatelessH265Decoder::Create(hevc_node, /*pool_size=*/28)) {
      V4L2WC_LOG(V4L2WC_INFO)
          << "v4l2wc: V4L2 stateless HEVC decoder on " << hevc_node << " "
          << config.coded_width << "x" << config.coded_height;
      return stateless;
    }
  }

#if V4L2WC_HAVE_VAAPI
  // VAAPI second: the desktop path, for H.264 and HEVC. The pool covers the
  // maximum DPB plus frames in flight to the compositor.
  if (config.codec_fourcc == V4L2_PIX_FMT_H264 ||
      config.codec_fourcc == V4L2_PIX_FMT_HEVC) {
    const char* node =
        (config.vaapi_render_node != nullptr && config.vaapi_render_node[0])
            ? config.vaapi_render_node
            : "/dev/dri/renderD128";
    std::unique_ptr<IDmaDecoder> vaapi =
        config.codec_fourcc == V4L2_PIX_FMT_HEVC
            ? std::unique_ptr<IDmaDecoder>(
                  VaapiH265Decoder::Create(node, /*pool_size=*/28))
            : std::unique_ptr<IDmaDecoder>(
                  VaapiH264Decoder::Create(node, /*pool_size=*/28));
    if (vaapi) {
      V4L2WC_LOG(V4L2WC_INFO) << "v4l2wc: VAAPI decoder " << config.coded_width
                              << "x" << config.coded_height;
      return vaapi;
    }
  }
#endif

  V4L2WC_LOG(V4L2WC_ERROR) << "v4l2wc: no decode engine available for "
                           << config.coded_width << "x" << config.coded_height;
  return nullptr;
}

}  // namespace v4l2wc

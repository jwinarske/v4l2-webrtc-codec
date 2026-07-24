// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Picks a decode engine for a stream and builds it. The V4L2 M2M engine (the
// embedded path) is tried first and VAAPI (the desktop path) second, which is
// the order the webrtc wrapper used before this selection was hoisted out, so
// a consumer that is not webrtc can choose an engine the same way.
#ifndef V4L2WC_SRC_DMA_DECODER_FACTORY_H_
#define V4L2WC_SRC_DMA_DECODER_FACTORY_H_

#include <cstdint>
#include <memory>

#include "src/dma_decoder.h"

namespace v4l2wc {

struct DmaDecoderConfig {
  // A V4L2 pixel format, e.g. V4L2_PIX_FMT_H264.
  std::uint32_t codec_fourcc = 0;
  std::uint32_t coded_width = 0;
  std::uint32_t coded_height = 0;

  // Device overrides; a null or empty value selects the library default.
  const char* v4l2_device = nullptr;        // default "/dev/video10"
  const char* vaapi_render_node = nullptr;  // default "/dev/dri/renderD128"
};

// Builds a decoder for `config`, or nullptr when no engine can serve it.
[[nodiscard]] std::unique_ptr<IDmaDecoder> CreateDmaDecoder(
    const DmaDecoderConfig& config);

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_DMA_DECODER_FACTORY_H_

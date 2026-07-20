# v4l2-webrtc-codec

Stateless (rkvdec2/rkvdec via the V4L2 request API) and stateful (Venus)
hardware `webrtc::VideoDecoder` factory that emits decoded frames as
**dmabuf descriptors** (`LwDmabufDescriptor`, zero copy) instead of CPU
pixel buffers.

## Packaging

Although this is a dedicated repo (own CI, own visl/vicodec harness,
standalone build against a webrtc checkout), the decoder ships **statically
absorbed into `libwebrtc.so`**, not as a standalone `.so`. It is pulled into
the webrtc checkout as a gclient custom dep and built as a GN target behind
`lw_enable_v4l2_codec=true`.

A standalone `.so` is rejected: the decoder subclasses `webrtc::VideoDecoder`
/ `webrtc::VideoFrameBuffer`, whose symbols are hidden inside `libwebrtc.so`
by design. Exporting a webrtc symbol allowlist would reopen the ABI fragility
the flat C boundary exists to close.

Selection is **by name** — `lw_factory_set_video_decoder_factory(h, "v4l2")`
over the string-keyed registry compiled into `libwebrtc.so` — or via the C
create entry below. Never dlopen.

## Public entry point

`include/v4l2wc/v4l2wc.h` — a self-contained C header. The only surface the
factory injection and the flat C API need:

```c
void* v4l2wc_create_factory(const V4l2WcConfig*);  /* webrtc::VideoDecoderFactory* */
```

Frames are emitted as the shared native-buffer class carrying
`LwDmabufDescriptor` (defined once in `libwebrtc/include/c/lw_video_sink.h`,
vendored here under `third_party/lw_abi/`).

## Codec scope (vs the m137 tree)

| Codec | Source | Notes |
|---|---|---|
| H.264 | **this repo** | stateless rkvdec2 / stateful Venus |
| H.265/HEVC | **this repo** | tree has transport only, no bundled codec; `parse/h265`, RPS tracking, `V4L2_CID_STATELESS_HEVC_*` |
| VP9 | libvpx (tree, software) | not this repo |
| AV1 | libaom/dav1d (tree, software) | not this repo; verify RK3588 AV1-core mainline driver status first |

No software fallback lives here — the composite factory in libwebrtc owns that.

## Pool lifecycle

The CAPTURE queue is the frame pool, sized `reorder + pipeline + 1`. Teardown
is **refcount-deferred** on resolution change: the new pool (`pool_generation`
+1, announced via `on_format`) is allocated immediately; the old pool is
`REQBUFS(0)`'d only when its last outstanding release lands. Two generations
may coexist during the transition. Dtor -> re-QBUF.

## Memory safety

The bitstream is attacker-controlled and reachable after the DTLS handshake.
Every `dpb[]` / ref-list / slice index is bounds-checked as a **stated
invariant, not an assert**. Mandatory CI:

- **visl** — draw-the-controls correctness oracle (stateless).
- **vicodec** — stateful-backend CI oracle: SOURCE_CHANGE choreography,
  CAPTURE setup sequencing, drain/EOS, timestamp round-trip.
- **fuzz gate** — libFuzzer over `parse/` (per-NAL corpus) and the DPB
  tracker (malicious `decode_params` sequences). visl is a correctness
  oracle, not a memory-safety one.

## Layout

```
include/v4l2wc/v4l2wc.h   public C entry point + V4l2WcConfig
src/                      factory, decoder, V4L2 request-API plumbing
parse/                    bitstream parsers (h264, h265) — fuzz targets
test/visl/                stateless CI harness
test/vicodec/             stateful CI harness
test/fuzz/                libFuzzer harnesses
third_party/lw_abi/       vendored lw_video_sink.h (pinned by LW_ABI_VERSION)
BUILD.gn                  GN target (lw_enable_v4l2_codec)
```

# vendored lw_abi

`lw_video_sink.h` is vendored verbatim from `jwinarske/libwebrtc`
(`include/c/lw_video_sink.h`), the single source of truth for the data-plane
ABI. Pinned by `LW_ABI_VERSION`. Do not edit here — update upstream and
re-vendor. CI asserts byte-identity against the pinned upstream revision.

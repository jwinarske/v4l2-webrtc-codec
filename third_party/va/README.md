<!--
SPDX-FileCopyrightText: 2026 Joel Winarske
SPDX-License-Identifier: MIT
-->

# vendored libva headers

The libva public headers (`va/`), vendored at **version 1.23.0** from
upstream <https://github.com/intel/libva> (the `va/` include directory of the
`1.23.0` release).

**Build-time only.** Nothing links against libva: the VAAPI engine resolves
every entry point with `dlopen`/`dlsym` at runtime and degrades gracefully
when the library or an H.264 VLD entrypoint is absent. Vendoring the headers
keeps the engine compilable inside the webrtc checkout and on hosts without
`libva-devel` installed, and pins the struct/enum layout the engine is
compiled against rather than inheriting whatever the build host happens to
have.

**License:** MIT. Each header carries its own permission notice; see
`va/va.h`. These files are upstream code and are excluded from this repo's
SPDX tagging.

Do not edit these headers. To update, re-vendor the `va/` directory from the
upstream release tag and bump the version recorded above.

# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# Crow buffers every request body in memory and has no size limit, so one
# request could exhaust ruxd's memory before it is even authenticated. The
# patch adds a compile-time cap, CROW_MAX_REQUEST_BODY (bytes; 0 = unlimited,
# the upstream behaviour), checked against Content-Length when the headers are
# complete and against the bytes read for a chunked body. ruxd defines it on
# ruxd_api_lib (PUBLIC, so every TU that includes Crow agrees on it).
_: _final: prev: {
  crow = prev.crow.overrideAttrs (old: {
    patches = (old.patches or []) ++ [./patches/crow-max-request-body.patch];
  });
}

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
#
# The second patch adds a header-phase check (`app.header_check(fn)`), run once
# a request's headers are parsed and before its body is read: ruxd decides
# authentication and authorization there, so an unauthenticated or forbidden
# request is answered (and its connection closed) without its body ever being
# buffered.
_: _final: prev: {
  crow = prev.crow.overrideAttrs (old: {
    patches =
      (old.patches or [])
      ++ [
        ./patches/crow-max-request-body.patch
        ./patches/crow-header-check.patch
      ];
  });
}

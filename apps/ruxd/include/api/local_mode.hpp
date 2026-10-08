// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pure helpers behind `ruxd --local` and the API server's token check.
//
// Framework-free on purpose (no Crow, no sockets) so the rules — which bind
// addresses count as loopback, how a request presents the token — are
// unit-tested in the light binary (tests/unit/ruxd_api/test_api_local_mode.cpp)
// rather than only through a listening server.

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace ruxd::api {

/// True when @p host only accepts connections from this machine: `localhost`,
/// any 127.0.0.0/8 literal, or `::1` (bracketed or not). Everything else —
/// `0.0.0.0`, `::`, a LAN address, a hostname — is reachable from elsewhere
/// and therefore needs an access token.
bool is_loopback_bind(std::string_view host);

/// Name of the cookie that carries the access token once a browser has
/// presented it with `?token=`: `ruxd_token_<port>`. Cookies are scoped to the
/// host, not the port, so a fixed name would let one ruxd's cookie shadow
/// another's (a restart with a new token, a second instance) and would hand
/// the token to every other service on the host.
std::string token_cookie_name(std::uint16_t port);

/// Outcome of checking a request's credentials against the access token.
struct TokenCheck {
  /// Some presented credential matched.
  bool ok = false;
  /// The `token` query parameter matched: the response should (re)set the
  /// cookie and, for a GET, redirect to the same URL without the token.
  bool via_query = false;
};

/// Check every credential a request presents — `Authorization: Bearer`, each
/// @p cookie_name cookie in @p cookie_header, the `token` query parameter —
/// and accept it if ANY matches, so a stale cookie never shadows a correct
/// header or query token.
TokenCheck check_token(std::string_view authorization_header,
                       std::string_view cookie_header,
                       std::string_view query_token,
                       std::string_view cookie_name, std::string_view expected);

/// @p raw_url (path plus query) with every @p key parameter removed; the `?`
/// goes too when nothing is left. Fragment-free input is assumed (browsers
/// never send one).
std::string strip_query_param(std::string_view raw_url, std::string_view key);

/// @p text with every query string — a `?` and what follows up to whitespace
/// — replaced by `?<redacted>`, for log lines that echo request URLs.
std::string redact_query_strings(std::string_view text);

/// True when @p origin is `http://<host>` or `https://<host>` for the request's
/// own Host header @p host (case-insensitive): a same-origin browser request.
bool origin_matches_host(std::string_view origin, std::string_view host);

/// Whether the request's Host header names this server (DNS-rebinding guard).
///
/// - loopback @p bind_address: only `localhost`, `127.0.0.1` or `[::1]`;
/// - wildcard bind (`0.0.0.0`, `::`): any host — such a bind always has an
///   access token, which a rebinding page cannot present;
/// - any other bind: the bind address itself or the host of an
///   @p allowed_origins entry.
/// The port is not compared. An empty Host is refused.
bool host_allowed(std::string_view host_header, std::string_view bind_address,
                  const std::vector<std::string> &allowed_origins);

/// Compare a presented token against the configured one in time that does not
/// depend on where they differ. An empty @p expected never matches.
bool token_matches(std::string_view presented, std::string_view expected);

} // namespace ruxd::api

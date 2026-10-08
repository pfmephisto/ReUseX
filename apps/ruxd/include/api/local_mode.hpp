// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pure helpers behind `ruxd --local` and the API server's token check.
//
// Framework-free on purpose (no Crow, no sockets) so the rules — which bind
// addresses count as loopback, which project a path names, how a request
// presents the token — are unit-tested in the light binary
// (tests/unit/ruxd_api/test_api_local_mode.cpp) rather than only through a
// listening server.

#include <filesystem>
#include <string>
#include <string_view>

namespace ruxd::api {

/// Name of the cookie that carries the local-mode access token once a browser
/// has presented it with `?token=`. HttpOnly, SameSite=Strict, Path=/.
inline constexpr std::string_view kTokenCookie = "ruxd_token";

/// True when @p host only accepts connections from this machine: `localhost`,
/// any 127.0.0.0/8 literal, or `::1` (bracketed or not). Everything else —
/// `0.0.0.0`, `::`, a LAN address, a hostname — is reachable from elsewhere
/// and therefore needs an access token.
bool is_loopback_bind(std::string_view host);

/// The project file `ruxd --local <target>` serves.
///
/// @p target is either a `.rux` file (returned as is; it is created on open
/// if it does not exist, like every project-writing command) or a directory
/// holding exactly one `.rux` file. Routes are single-project until cases
/// arrive (spec phase S2), so a directory with several `.rux` files is an
/// error that names them, not a silent pick.
/// @throws std::runtime_error with a user-facing message.
std::filesystem::path
resolve_local_project(const std::filesystem::path &target);

/// The token a request presents, from (in order) `Authorization: Bearer <t>`,
/// the #kTokenCookie cookie in @p cookie_header, or the `token` query
/// parameter. Empty when it presents none.
std::string presented_token(std::string_view authorization_header,
                            std::string_view cookie_header,
                            std::string_view query_token);

/// Compare a presented token against the configured one in time that does not
/// depend on where they differ. An empty @p expected never matches.
bool token_matches(std::string_view presented, std::string_view expected);

} // namespace ruxd::api

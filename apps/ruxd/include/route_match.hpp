// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Security primitives shared by the endpoint registry and the auth middleware:
// matching a registered Crow route *rule* against a concrete request path, and
// comparing a presented bearer token against the configured one without
// leaking its bytes through timing.
//
// Deliberately free functions with no Crow (or any server) dependency so they
// can be unit-tested without standing up an HTTP app.

#include <string_view>

namespace ruxd {

// Strip the query string / fragment from a request URL, leaving just the path.
// Crow already hands us a parameter-free `req.url`, but callers may pass a raw
// URL and the matcher must never treat "?x=1" as part of a path segment.
[[nodiscard]] std::string_view request_path(std::string_view url);

// Whether a registered Crow route rule matches a concrete request path.
//
// Crow rules may contain placeholder segments — `<int>`, `<uint>`, `<double>`,
// `<str>`, `<string>`, `<path>`. Everything of the form `<...>` is treated as a
// wildcard: `<path>` matches one *or more* trailing segments (it is Crow's
// catch-all), any other placeholder matches exactly one segment. No type
// checking is performed — the point is to find the rule a request would be
// routed to, and a rule that matches too eagerly can only ever make auth
// *stricter* (see EndpointRegistry::requires_auth, which requires a token as
// soon as any matching rule is protected).
//
// Empty segments are ignored, so trailing slashes and repeated slashes are
// normalized away: "/health", "/health/" and "//health" all match "/health".
// Comparison of literal segments is case-sensitive, matching Crow's router; a
// case-mismatched request therefore matches nothing and — by the fail-closed
// default — requires a token.
//
// Placeholders embedded inside a larger segment (e.g. "/f/pre<int>") are not
// recognized as wildcards and simply won't match, which again errs towards
// requiring auth. ruxd has no such rules today.
[[nodiscard]] bool route_pattern_matches(std::string_view pattern,
                                         std::string_view url);

// Case-insensitive comparison of HTTP method names ("GET" == "get").
[[nodiscard]] bool http_method_equals(std::string_view a, std::string_view b);

// Compare two secrets without an early return on the first differing byte.
//
// The number of iterations depends only on `presented.size()` — public
// information — never on where the two inputs start to differ. When the lengths
// disagree the loop still runs over the whole presented value (folding
// `expected` cyclically as a same-length dummy) and the result is forced to
// false, so a wrong-length token costs the same as a wrong-value one.
[[nodiscard]] bool constant_time_equals(std::string_view presented,
                                        std::string_view expected);

} // namespace ruxd

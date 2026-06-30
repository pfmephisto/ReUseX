// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Bearer-token authentication middleware. For every request it consults the
// endpoint registry: if the matched route is flagged requires_auth, it demands
// an `Authorization: Bearer <token>` header matching the configured token and
// short-circuits with 401 otherwise. Routes that don't require auth pass
// through untouched.
//
// If no token is configured, auth is disabled (every route passes) — convenient
// for local development; main() logs a warning in that case.

#include "endpoints.hpp"

#include <crow.h>

#include <string>
#include <string_view>
#include <utility>

namespace ruxd {

struct BearerAuthMiddleware {
  // Per-request state. `authenticated` is true when a valid bearer token was
  // presented (or auth is disabled — a trusted/open environment). Handlers can
  // read it via app.get_context<BearerAuthMiddleware>(req) to vary their output
  // by authentication, e.g. /health returns detail only when authenticated.
  struct context {
    bool authenticated = false;
  };

  // Configured once at startup (after routes are registered).
  const EndpointRegistry *registry = nullptr;
  std::string token; // expected bearer token; empty disables auth

  void configure(const EndpointRegistry *reg, std::string tok) {
    registry = reg;
    token = std::move(tok);
  }

  [[nodiscard]] bool has_valid_token(const crow::request &req) const {
    const std::string &header = req.get_header_value("Authorization");
    constexpr std::string_view prefix = "Bearer ";
    const std::string_view value(header);
    return value.size() > prefix.size() &&
           value.substr(0, prefix.size()) == prefix &&
           value.substr(prefix.size()) == token;
  }

  void before_handle(crow::request &req, crow::response &res, context &ctx) {
    // Record auth status for every request (public routes included) so handlers
    // can vary output. Authenticated means a *valid* bearer token was
    // presented — disabled auth (no token configured) is NOT authenticated, so
    // privileged detail (e.g. /health) is never exposed without a correct
    // token, even to requests carrying a wrong one.
    ctx.authenticated = !token.empty() && has_valid_token(req);

    if (token.empty() || registry == nullptr) {
      return; // auth disabled — no enforcement
    }
    if (!registry->requires_auth(crow::method_name(req.method), req.url)) {
      return; // public route — flag set above, no enforcement
    }
    if (ctx.authenticated) {
      return; // valid token on a protected route
    }

    res.code = 401;
    res.set_header("WWW-Authenticate", "Bearer");
    res.set_header("Content-Type", "application/json");
    res.body =
        R"({"status":"unauthorized","message":"missing or invalid bearer token"})";
    res.end();
  }

  void after_handle(crow::request &, crow::response &, context &) {}
};

// The concrete Crow application type for ruxd, with the auth middleware
// installed. Used throughout instead of crow::SimpleApp.
using App = crow::App<BearerAuthMiddleware>;

} // namespace ruxd

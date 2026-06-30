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
  struct context {};

  // Configured once at startup (after routes are registered).
  const EndpointRegistry *registry = nullptr;
  std::string token; // expected bearer token; empty disables auth

  void configure(const EndpointRegistry *reg, std::string tok) {
    registry = reg;
    token = std::move(tok);
  }

  void before_handle(crow::request &req, crow::response &res, context &) {
    if (token.empty() || registry == nullptr) {
      return; // auth disabled
    }
    if (!registry->requires_auth(crow::method_name(req.method), req.url)) {
      return; // public route
    }

    const std::string &header = req.get_header_value("Authorization");
    constexpr std::string_view prefix = "Bearer ";
    const std::string_view value(header);
    if (value.size() > prefix.size() && value.substr(0, prefix.size()) == prefix &&
        value.substr(prefix.size()) == token) {
      return; // authorized
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

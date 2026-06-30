// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Single source of truth for ruxd's HTTP routes. Handlers register through
// add_route() (declared in handlers.hpp), which both attaches the route to
// Crow and records its metadata here. The registry then backs GET /endpoints
// and GET /openapi.json, so the docs can never drift from the registered
// routes, and the Bearer-auth middleware consults requires_auth() to decide
// which paths to protect.

#include <crow.h>
#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

namespace ruxd {

// A possible HTTP response from an endpoint.
struct Response {
  int code;                // e.g. 200, 503
  std::string description; // human-readable meaning
};

// Metadata describing one HTTP endpoint.
struct Endpoint {
  std::string method;             // "GET", "POST", ...
  std::string path;               // "/health"
  std::string summary;            // human-readable description
  bool requires_auth = false;     // requires a valid Bearer token
  std::vector<Response> responses;// declared response codes (401 auto-added
                                  // for authenticated endpoints)
};

// Collects endpoint metadata as routes are registered and renders it as
// JSON (/endpoints) or an OpenAPI 3.1 document (/openapi.json).
class EndpointRegistry {
public:
  void add(const Endpoint &e) { endpoints_.push_back(e); }
  [[nodiscard]] const std::vector<Endpoint> &endpoints() const {
    return endpoints_;
  }

  // Whether the route matching (method, path) requires authentication.
  [[nodiscard]] bool requires_auth(const std::string &method,
                                   const std::string &path) const;

  [[nodiscard]] nlohmann::json to_json() const;
  [[nodiscard]] nlohmann::json to_openapi() const;

private:
  std::vector<Endpoint> endpoints_;
};

using RouteHandler = std::function<crow::response(const crow::request &)>;

} // namespace ruxd

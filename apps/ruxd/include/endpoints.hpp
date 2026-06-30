// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Single source of truth for ruxd's HTTP routes. Handlers register through
// add_route(), which both attaches the route to Crow and records its metadata
// in an EndpointRegistry. The registry then backs GET /endpoints and
// GET /openapi.json, so the docs can never drift from the registered routes.
//
// Each endpoint carries a `requires_auth` flag. No middleware enforces it yet,
// but it is surfaced in /endpoints and emitted as an OpenAPI `security`
// requirement; a future Bearer-auth middleware can use the same registry to
// decide which paths to protect.

#include <crow.h>
#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

namespace ruxd {

// Metadata describing one HTTP endpoint.
struct Endpoint {
  std::string method;         // "GET", "POST", ...
  std::string path;           // "/health"
  std::string summary;        // human-readable description
  bool requires_auth = false; // requires a valid Bearer token
};

// Collects endpoint metadata as routes are registered and renders it as
// JSON (/endpoints) or an OpenAPI 3.1 document (/openapi.json).
class EndpointRegistry {
public:
  void add(const Endpoint &e) { endpoints_.push_back(e); }
  [[nodiscard]] const std::vector<Endpoint> &endpoints() const {
    return endpoints_;
  }

  [[nodiscard]] nlohmann::json to_json() const;
  [[nodiscard]] nlohmann::json to_openapi() const;

private:
  std::vector<Endpoint> endpoints_;
};

using RouteHandler = std::function<crow::response(const crow::request &)>;

// Register a route with Crow and record its metadata in the registry.
void add_route(crow::SimpleApp &app, EndpointRegistry &reg, Endpoint meta,
               RouteHandler handler);

} // namespace ruxd

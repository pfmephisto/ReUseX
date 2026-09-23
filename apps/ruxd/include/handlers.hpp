// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Route registration for the ruxd HTTP service worker.
//
// Each feature area lives in its own translation unit under src/handlers/ and
// exposes a register_* function that attaches its routes to the shared App via
// add_route() (which also records each route in the EndpointRegistry). main()
// calls each registrar once at startup. Add a new handler file + a declaration
// here as the API grows — this keeps the route surface discoverable.

#include "auth.hpp"      // App alias + auth middleware
#include "endpoints.hpp" // Endpoint, EndpointRegistry, RouteHandler

#include <crow.h>
#include <nlohmann/json.hpp>

#include <utility>

namespace reusex {
class ProjectDB;
} // namespace reusex

namespace ruxd {

struct Clients;

// Build a JSON crow::response with the Content-Type header set.
inline crow::response json_response(crow::status code,
                                    const nlohmann::json &body) {
  crow::response res(code, body.dump());
  res.set_header("Content-Type", "application/json");
  return res;
}

// Record an endpoint's metadata in the registry and return the Crow HTTP method
// bit for it. Used by add_route and the templated add_route_dynamic below.
crow::HTTPMethod register_endpoint(EndpointRegistry &reg, const Endpoint &meta);

// Register a route with Crow and record its metadata in the registry.
void add_route(App &app, EndpointRegistry &reg, Endpoint meta,
               RouteHandler handler);

// Register a route whose path contains placeholder segments (e.g. `<string>`)
// and record its metadata. The handler is forwarded to Crow unwrapped so Crow
// can deduce its placeholder arity — a `std::function` defeats that deduction,
// so callers pass a plain lambda taking (const crow::request&, args...).
template <typename Func>
void add_route_dynamic(App &app, EndpointRegistry &reg, Endpoint meta,
                       Func &&handler) {
  const crow::HTTPMethod method = register_endpoint(reg, meta);
  app.route_dynamic(meta.path).methods(method)(std::forward<Func>(handler));
}

// GET / (liveness) and GET /health (overview of every configured backend).
void register_health_routes(App &app, EndpointRegistry &reg, Clients &clients);

// Point cloud segmentation — POST /segment/planes (stub for now).
void register_segment_routes(App &app, EndpointRegistry &reg);

// GET /endpoints and GET /openapi.json — generated from the registry.
void register_meta_routes(App &app, EndpointRegistry &reg);

// Material passport CRUD + thumbnail routes (#414). POST/DELETE /materials,
// GET /materials/{guid}, GET/PUT /materials/{guid}/thumbnail.
void register_material_routes(App &app, EndpointRegistry &reg,
                              reusex::ProjectDB &db);

// Material column-definition CRUD (#415): GET/POST /material-columns and
// PATCH/DELETE /material-columns/{id}.
void register_material_column_routes(App &app, EndpointRegistry &reg,
                                     reusex::ProjectDB &db);

// Catchall handler returning a JSON 404 for unmatched routes.
void register_not_found_handler(App &app);

} // namespace ruxd

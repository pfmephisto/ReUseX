// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <endpoints.hpp>

#include <reusex/core/version.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace ruxd {

namespace {
crow::HTTPMethod to_method(const std::string &m) {
  if (m == "GET") return crow::HTTPMethod::Get;
  if (m == "POST") return crow::HTTPMethod::Post;
  if (m == "PUT") return crow::HTTPMethod::Put;
  if (m == "DELETE") return crow::HTTPMethod::Delete;
  if (m == "PATCH") return crow::HTTPMethod::Patch;
  if (m == "HEAD") return crow::HTTPMethod::Head;
  if (m == "OPTIONS") return crow::HTTPMethod::Options;
  throw std::invalid_argument(
      fmt::format("ruxd: unsupported HTTP method '{}'", m));
}
} // namespace

void add_route(crow::SimpleApp &app, EndpointRegistry &reg, Endpoint meta,
               RouteHandler handler) {
  const crow::HTTPMethod method = to_method(meta.method);
  reg.add(meta);
  app.route_dynamic(meta.path).methods(method)(std::move(handler));
}

nlohmann::json EndpointRegistry::to_json() const {
  nlohmann::json arr = nlohmann::json::array();
  for (const Endpoint &e : endpoints_) {
    arr.push_back({
        {"method", e.method},
        {"path", e.path},
        {"summary", e.summary},
        {"requires_auth", e.requires_auth},
    });
  }
  return {{"endpoints", arr}};
}

nlohmann::json EndpointRegistry::to_openapi() const {
  nlohmann::json paths = nlohmann::json::object();
  for (const Endpoint &e : endpoints_) {
    std::string verb = e.method;
    std::transform(verb.begin(), verb.end(), verb.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    nlohmann::json operation{
        {"summary", e.summary},
        {"responses", {{"200", {{"description", "Success"}}}}},
    };
    if (e.requires_auth) {
      operation["security"] =
          nlohmann::json::array({{{"bearerAuth", nlohmann::json::array()}}});
    }
    paths[e.path][verb] = operation;
  }

  return {
      {"openapi", "3.1.0"},
      {"info",
       {{"title", "ruxd — ReUseX service worker"},
        {"version", reusex::core::VERSION}}},
      {"components",
       {{"securitySchemes",
         {{"bearerAuth", {{"type", "http"}, {"scheme", "bearer"}}}}}}},
      {"paths", paths},
  };
}

} // namespace ruxd

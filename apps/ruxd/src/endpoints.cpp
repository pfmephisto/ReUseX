// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp> // App + add_route

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

// The response codes an endpoint can actually return: those it declared, plus
// 401 for authenticated endpoints (added by the auth middleware), with a 200
// fallback when nothing was declared.
std::vector<Response> effective_responses(const Endpoint &e) {
  std::vector<Response> r = e.responses;
  if (r.empty()) {
    r.push_back({200, "Success"});
  }
  if (e.requires_auth &&
      std::none_of(r.begin(), r.end(),
                   [](const Response &x) { return x.code == 401; })) {
    r.push_back({401, "Missing or invalid bearer token"});
  }
  return r;
}
} // namespace

void add_route(App &app, EndpointRegistry &reg, Endpoint meta,
               RouteHandler handler) {
  const crow::HTTPMethod method = to_method(meta.method);
  reg.add(meta);
  app.route_dynamic(meta.path).methods(method)(std::move(handler));
}

bool EndpointRegistry::requires_auth(const std::string &method,
                                     const std::string &path) const {
  for (const Endpoint &e : endpoints_) {
    if (e.method == method && e.path == path) {
      return e.requires_auth;
    }
  }
  return false;
}

nlohmann::json EndpointRegistry::to_json() const {
  nlohmann::json arr = nlohmann::json::array();
  for (const Endpoint &e : endpoints_) {
    nlohmann::json responses = nlohmann::json::array();
    for (const Response &r : effective_responses(e)) {
      responses.push_back({{"code", r.code}, {"description", r.description}});
    }
    arr.push_back({
        {"method", e.method},
        {"path", e.path},
        {"summary", e.summary},
        {"requires_auth", e.requires_auth},
        {"responses", responses},
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

    nlohmann::json responses = nlohmann::json::object();
    for (const Response &r : effective_responses(e)) {
      responses[std::to_string(r.code)] = {{"description", r.description}};
    }

    nlohmann::json operation{
        {"summary", e.summary},
        {"responses", responses},
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

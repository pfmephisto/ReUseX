// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Material passport CRUD + thumbnail routes for the Notion-like material editor
// (#414). These wrap ProjectDB directly; ProjectDB is not thread-safe, so the
// server must serialise access to it (a single shared instance is passed in by
// reference here — the caller owns the lifetime and concurrency guarantees).

#include <handlers.hpp>

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/logging.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <string>
#include <utility>
#include <vector>

namespace ruxd {

namespace {

// Current time as an ISO 8601 UTC timestamp, e.g. "2026-09-22T12:34:56Z".
std::string now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return std::string(buf);
}

crow::response error_json(crow::status code, const std::string &message) {
  return json_response(code, {{"error", message}});
}

// Move a built response into Crow's out-parameter and finish the request.
// Crow 1.3's value-returning handler form is unreliable for dynamic routes that
// carry URL placeholders, so those handlers take (req, res, args...) and write
// through this helper instead of returning.
void finish(crow::response &res, crow::response &&built) {
  res = std::move(built);
  res.end();
}

} // namespace

void register_material_routes(App &app, EndpointRegistry &reg,
                              reusex::ProjectDB &db) {
  // POST /materials — create a new blank passport.
  add_route(app, reg,
            {"POST",
             "/materials",
             "Create a new blank material passport",
             true,
             {{201, "Passport created"}}},
            [&db](const crow::request &) {
              try {
                const std::string guid = reusex::core::generate_guid();
                const std::string created_at = now_iso8601();

                reusex::core::MaterialPassport passport;
                passport.metadata.document_guid = guid;
                passport.metadata.creation_date = created_at;
                passport.metadata.version_number = "0.1.0";

                db.add_material_passport(passport, "");

                return json_response(crow::status::CREATED,
                                     {{"guid", guid},
                                      {"id", guid},
                                      {"properties", nlohmann::json::object()},
                                      {"has_thumbnail", false},
                                      {"created_at", created_at},
                                      {"version_number", "0.1.0"}});
              } catch (const std::exception &e) {
                reusex::core::error("POST /materials failed: {}", e.what());
                return error_json(crow::status::INTERNAL_SERVER_ERROR,
                                  e.what());
              }
            });

  // GET /materials/{guid} — one passport with its stored properties.
  add_route_dynamic(
      app, reg,
      {"GET",
       "/materials/<string>",
       "One material passport with its stored properties",
       true,
       {{200, "Passport"}, {404, "No such passport"}}},
      [&db](const crow::request &, crow::response &res, std::string guid) {
        try {
          const auto guids = db.list_passport_guids();
          if (std::find(guids.begin(), guids.end(), guid) == guids.end()) {
            finish(res, error_json(crow::status::NOT_FOUND,
                                   "material passport not found: " + guid));
            return;
          }

          const auto properties = db.passport_stored_properties(guid);
          nlohmann::json props = nlohmann::json::object();
          for (const auto &[key, value] : properties)
            props[key] = value;

          nlohmann::json out{
              {"guid", guid},
              {"property_count", properties.size()},
              {"properties", std::move(props)},
              {"has_thumbnail", db.material_thumbnail(guid).has_value()}};

          for (const auto &material : db.project_summary().materials) {
            if (material.guid != guid)
              continue;
            out["id"] = material.id;
            out["created_at"] = material.created_at;
            out["version_number"] = material.version_number;
            break;
          }

          if (auto node_id = db.passport_linked_node_id(guid))
            out["linked_node_id"] = *node_id;

          finish(res, json_response(crow::status::OK, out));
        } catch (const std::exception &e) {
          reusex::core::error("GET /materials/{} failed: {}", guid, e.what());
          finish(res,
                 error_json(crow::status::INTERNAL_SERVER_ERROR, e.what()));
        }
      });

  // DELETE /materials/{guid} — remove a passport.
  add_route_dynamic(
      app, reg,
      {"DELETE",
       "/materials/<string>",
       "Delete a material passport",
       true,
       {{204, "Passport deleted"}, {404, "No such passport"}}},
      [&db](const crow::request &, crow::response &res, std::string guid) {
        try {
          db.delete_material_passport(guid);
          finish(res, crow::response(crow::status::NO_CONTENT));
        } catch (const std::exception &e) {
          reusex::core::warn("DELETE /materials/{} failed: {}", guid, e.what());
          finish(res, error_json(crow::status::NOT_FOUND, e.what()));
        }
      });

  // GET /materials/{guid}/thumbnail — the stored thumbnail image bytes.
  add_route_dynamic(
      app, reg,
      {"GET",
       "/materials/<string>/thumbnail",
       "Fetch a material's thumbnail image",
       true,
       {{200, "Thumbnail image"}, {404, "No thumbnail stored"}}},
      [&db](const crow::request &, crow::response &res, std::string guid) {
        try {
          const auto thumb = db.material_thumbnail(guid);
          if (!thumb.has_value()) {
            finish(res,
                   error_json(crow::status::NOT_FOUND,
                              "no thumbnail stored for material: " + guid));
            return;
          }

          const auto &[blob, mime] = *thumb;
          crow::response out(crow::status::OK);
          out.body.assign(reinterpret_cast<const char *>(blob.data()),
                          blob.size());
          out.set_header("Content-Type", mime);
          finish(res, std::move(out));
        } catch (const std::exception &e) {
          reusex::core::error("GET /materials/{}/thumbnail failed: {}", guid,
                              e.what());
          finish(res,
                 error_json(crow::status::INTERNAL_SERVER_ERROR, e.what()));
        }
      });

  // PUT /materials/{guid}/thumbnail — upload / replace the thumbnail image.
  add_route_dynamic(
      app, reg,
      {"PUT",
       "/materials/<string>/thumbnail",
       "Upload or replace a material's thumbnail image",
       true,
       {{204, "Thumbnail stored"}}},
      [&db](const crow::request &req, crow::response &res, std::string guid) {
        try {
          std::string mime = req.get_header_value("Content-Type");
          if (mime.empty())
            mime = "image/jpeg";

          std::vector<std::uint8_t> bytes(req.body.begin(), req.body.end());
          db.set_material_thumbnail(guid, bytes, mime);

          finish(res, crow::response(crow::status::NO_CONTENT));
        } catch (const std::exception &e) {
          reusex::core::error("PUT /materials/{}/thumbnail failed: {}", guid,
                              e.what());
          finish(res,
                 error_json(crow::status::INTERNAL_SERVER_ERROR, e.what()));
        }
      });
}

} // namespace ruxd

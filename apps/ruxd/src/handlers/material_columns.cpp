// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// User-defined column definitions for the Notion-like material editor (#415).
// CRUD over ProjectDB::PropertyDefinition, backed by the
// material_property_definitions table (schema v18). ProjectDB is not
// thread-safe; the caller serialises access to the shared instance passed here.

#include <handlers.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ruxd {

namespace {

constexpr std::array<std::string_view, 5> kValidTypes = {
    "text", "number", "date", "boolean", "select"};

bool is_valid_type(const std::string &type) {
  return std::find(kValidTypes.begin(), kValidTypes.end(), type) !=
         kValidTypes.end();
}

nlohmann::json definition_json(const reusex::ProjectDB::PropertyDefinition &d) {
  return nlohmann::json{{"id", d.id},
                        {"name", d.name},
                        {"type", d.type},
                        {"options", d.options},
                        {"sort_order", d.sort_order}};
}

crow::response error_json(crow::status code, const std::string &message) {
  return json_response(code, {{"error", message}});
}

// Move a built response into Crow's out-parameter and finish. Used by the
// dynamic (URL-placeholder) routes; see the note in materials.cpp.
void finish(crow::response &res, crow::response &&built) {
  res = std::move(built);
  res.end();
}

} // namespace

void register_material_column_routes(App &app, EndpointRegistry &reg,
                                     reusex::ProjectDB &db) {
  // GET /material-columns — all column definitions.
  add_route(
      app, reg,
      {"GET",
       "/material-columns",
       "List material column definitions",
       true,
       {{200, "Column definitions"}}},
      [&db](const crow::request &) {
        try {
          nlohmann::json arr = nlohmann::json::array();
          for (const auto &def : db.list_property_definitions())
            arr.push_back(definition_json(def));
          return json_response(crow::status::OK, arr);
        } catch (const std::exception &e) {
          reusex::core::error("GET /material-columns failed: {}", e.what());
          return error_json(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // POST /material-columns — create a column definition.
  add_route(
      app, reg,
      {"POST",
       "/material-columns",
       "Create a material column definition",
       true,
       {{201, "Column created"}, {400, "Invalid request"}}},
      [&db](const crow::request &req) {
        nlohmann::json body = nlohmann::json::parse(req.body, nullptr,
                                                    /*allow_exceptions=*/false);
        if (body.is_discarded() || !body.is_object())
          return error_json(crow::status::BAD_REQUEST,
                            "request body must be a JSON object");
        if (!body.contains("name") || !body["name"].is_string())
          return error_json(crow::status::BAD_REQUEST,
                            "'name' is required and must be a string");
        if (!body.contains("type") || !body["type"].is_string())
          return error_json(crow::status::BAD_REQUEST,
                            "'type' is required and must be a string");

        const std::string name = body["name"].get<std::string>();
        const std::string type = body["type"].get<std::string>();
        if (!is_valid_type(type))
          return error_json(
              crow::status::BAD_REQUEST,
              "'type' must be one of text/number/date/boolean/select");

        std::vector<std::string> options;
        if (body.contains("options") && body["options"].is_array()) {
          for (const auto &opt : body["options"])
            if (opt.is_string())
              options.push_back(opt.get<std::string>());
        }
        int sort_order = 0;
        if (body.contains("sort_order") &&
            body["sort_order"].is_number_integer())
          sort_order = body["sort_order"].get<int>();

        try {
          const std::string id =
              db.add_property_definition(name, type, options, sort_order);
          reusex::ProjectDB::PropertyDefinition created;
          created.id = id;
          created.name = name;
          created.type = type;
          created.options = options;
          created.sort_order = sort_order;
          return json_response(crow::status::CREATED, definition_json(created));
        } catch (const std::exception &e) {
          reusex::core::error("POST /material-columns failed: {}", e.what());
          return error_json(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // PATCH /material-columns/{id} — sparse update.
  add_route_dynamic(
      app, reg,
      {"PATCH",
       "/material-columns/<string>",
       "Update a material column definition",
       true,
       {{200, "Column updated"},
        {400, "Invalid request"},
        {404, "No such column"}}},
      [&db](const crow::request &req, crow::response &res, std::string id) {
        nlohmann::json body = nlohmann::json::parse(req.body, nullptr,
                                                    /*allow_exceptions=*/false);
        if (body.is_discarded() || !body.is_object()) {
          finish(res, error_json(crow::status::BAD_REQUEST,
                                 "request body must be a JSON object"));
          return;
        }

        try {
          const auto defs = db.list_property_definitions();
          const auto it =
              std::find_if(defs.begin(), defs.end(),
                           [&](const auto &d) { return d.id == id; });
          if (it == defs.end()) {
            finish(res, error_json(crow::status::NOT_FOUND,
                                   "column definition not found: " + id));
            return;
          }

          // Apply the sparse patch on top of the current values.
          std::string name = it->name;
          std::string type = it->type;
          std::vector<std::string> options = it->options;
          int sort_order = it->sort_order;

          if (body.contains("name")) {
            if (!body["name"].is_string()) {
              finish(res, error_json(crow::status::BAD_REQUEST,
                                     "'name' must be a string"));
              return;
            }
            name = body["name"].get<std::string>();
          }
          if (body.contains("type")) {
            if (!body["type"].is_string()) {
              finish(res, error_json(crow::status::BAD_REQUEST,
                                     "'type' must be a string"));
              return;
            }
            type = body["type"].get<std::string>();
            if (!is_valid_type(type)) {
              finish(res, error_json(crow::status::BAD_REQUEST,
                                     "'type' must be one of "
                                     "text/number/date/boolean/select"));
              return;
            }
          }
          if (body.contains("options")) {
            if (!body["options"].is_array()) {
              finish(res, error_json(crow::status::BAD_REQUEST,
                                     "'options' must be an array"));
              return;
            }
            options.clear();
            for (const auto &opt : body["options"])
              if (opt.is_string())
                options.push_back(opt.get<std::string>());
          }
          if (body.contains("sort_order")) {
            if (!body["sort_order"].is_number_integer()) {
              finish(res, error_json(crow::status::BAD_REQUEST,
                                     "'sort_order' must be an integer"));
              return;
            }
            sort_order = body["sort_order"].get<int>();
          }

          db.update_property_definition(id, name, type, options, sort_order);

          reusex::ProjectDB::PropertyDefinition updated;
          updated.id = id;
          updated.name = name;
          updated.type = type;
          updated.options = options;
          updated.sort_order = sort_order;
          finish(res,
                 json_response(crow::status::OK, definition_json(updated)));
        } catch (const std::exception &e) {
          reusex::core::error("PATCH /material-columns/{} failed: {}", id,
                              e.what());
          finish(res,
                 error_json(crow::status::INTERNAL_SERVER_ERROR, e.what()));
        }
      });

  // DELETE /material-columns/{id}.
  add_route_dynamic(
      app, reg,
      {"DELETE",
       "/material-columns/<string>",
       "Delete a material column definition",
       true,
       {{204, "Column deleted"}, {404, "No such column"}}},
      [&db](const crow::request &, crow::response &res, std::string id) {
        try {
          db.delete_property_definition(id);
          finish(res, crow::response(crow::status::NO_CONTENT));
        } catch (const std::exception &e) {
          reusex::core::warn("DELETE /material-columns/{} failed: {}", id,
                             e.what());
          finish(res, error_json(crow::status::NOT_FOUND, e.what()));
        }
      });
}

} // namespace ruxd

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd routes for CSV export and named export-template CRUD (#459).
//
//   GET    /exports/csv                       — stream CSV (optional ?columns=)
//   GET    /export-templates                  — list all templates
//   POST   /export-templates                  — create a template
//   GET    /export-templates/<int>            — fetch one template
//   PATCH  /export-templates/<int>            — update name / config
//   DELETE /export-templates/<int>            — remove a template

#include <handlers.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/geometry/BuildingComponent.hpp>
#include <reusex/geometry/component_persistence.hpp>

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace ruxd {

namespace {

// --- CSV generation helpers (mirrored from apps/rux/src/export/csv.cpp) -----

const std::vector<std::string> k_component_cols = {
    "component_name",  "component_type",  "confidence",   "parent_id",
    "notes",           "source_instance", "window_style", "window_pane_count",
    "window_operable", "door_style",      "door_swing"};

const std::vector<std::string> k_passport_meta_cols = {
    "linked_instance", "passport_version_number", "passport_creation_date",
    "passport_revision_date", "passport_version_date"};

std::string csv_escape(const std::string &field) {
  if (field.find_first_of(",\"\n\r") == std::string::npos)
    return field;
  std::string out = "\"";
  for (char c : field) {
    if (c == '"')
      out += "\"\"";
    else
      out += c;
  }
  out += "\"";
  return out;
}

std::string passport_column(const std::string &section_en,
                            const std::string &prop_name) {
  return section_en + " / " + prop_name;
}

std::vector<std::string>
passport_property_columns(const nlohmann::json &sections) {
  std::vector<std::string> cols;
  std::set<std::string> seen;
  for (const auto &section : sections) {
    std::string sec = section.value("nameEN", std::string{});
    for (const auto &prop : section.at("properties")) {
      std::string col = passport_column(sec, prop.value("name", std::string{}));
      if (seen.insert(col).second)
        cols.push_back(col);
    }
  }
  return cols;
}

void write_row(std::ostream &os, const std::vector<std::string> &header,
               const std::map<std::string, std::string> &row) {
  for (size_t i = 0; i < header.size(); ++i) {
    if (i)
      os << ',';
    auto it = row.find(header[i]);
    os << csv_escape(it == row.end() ? std::string{} : it->second);
  }
  os << '\n';
}

// Build full header; optionally restrict to the caller-supplied column set.
// Unknown column names in the selection are silently dropped (the client may
// pass stale template config).
std::vector<std::string> build_header(const nlohmann::json &blank_template,
                                      const std::vector<std::string> &select) {
  std::vector<std::string> all = {"kind", "id"};
  all.insert(all.end(), k_component_cols.begin(), k_component_cols.end());
  all.insert(all.end(), k_passport_meta_cols.begin(),
             k_passport_meta_cols.end());
  const auto prop_cols =
      passport_property_columns(blank_template.at("sections"));
  all.insert(all.end(), prop_cols.begin(), prop_cols.end());

  if (select.empty())
    return all;

  // Preserve declaration order from `select`; keep only valid columns.
  const std::set<std::string> valid(all.begin(), all.end());
  std::vector<std::string> out;
  for (const auto &col : select) {
    if (valid.count(col))
      out.push_back(col);
  }
  return out;
}

// Generate the full CSV into `os` for the given `db`, restricted to `header`.
void generate_csv(std::ostream &os, reusex::ProjectDB &db,
                  const std::vector<std::string> &header) {
  // Header row.
  for (size_t i = 0; i < header.size(); ++i) {
    if (i)
      os << ',';
    os << csv_escape(header[i]);
  }
  os << '\n';

  // --- Building components ---
  auto component_names = db.list_building_components();
  for (const auto &name : component_names) {
    auto c = reusex::geometry::building_component(db, name);
    std::map<std::string, std::string> row;
    row["kind"] = "component";
    row["id"] = c.guid;
    row["component_name"] = c.name;
    row["component_type"] = std::string(reusex::geometry::to_string(c.type));
    if (c.confidence >= 0.0)
      row["confidence"] = std::to_string(c.confidence);
    if (c.parent_id >= 0)
      row["parent_id"] = std::to_string(c.parent_id);
    row["notes"] = c.notes;
    row["source_instance"] = c.source_instance_guid;

    std::visit(
        [&](auto &&d) {
          using T = std::decay_t<decltype(d)>;
          if constexpr (std::is_same_v<T, reusex::geometry::WindowData>) {
            row["window_style"] = d.style;
            row["window_pane_count"] = std::to_string(d.pane_count);
            row["window_operable"] = d.operable ? "true" : "false";
          } else if constexpr (std::is_same_v<T, reusex::geometry::DoorData>) {
            row["door_style"] = d.style;
            row["door_swing"] = d.swing;
          }
        },
        c.data);

    write_row(os, header, row);
  }

  // --- Material passports ---
  std::map<std::string, std::string> guid_to_instance;
  const std::string inst_cloud = "instances";
  if (db.has_point_cloud(inst_cloud)) {
    for (const auto &[iid, g] : db.instance_materials(inst_cloud))
      guid_to_instance[g] = inst_cloud + "#" + std::to_string(iid);
  }

  auto passports = db.all_material_passports();
  for (const auto &p : passports) {
    std::map<std::string, std::string> row;
    row["kind"] = "passport";
    row["id"] = p.metadata.document_guid;
    if (auto it = guid_to_instance.find(p.metadata.document_guid);
        it != guid_to_instance.end())
      row["linked_instance"] = it->second;
    row["passport_version_number"] = p.metadata.version_number;
    row["passport_creation_date"] = p.metadata.creation_date;
    row["passport_revision_date"] = p.metadata.revision_date;
    row["passport_version_date"] = p.metadata.version_date;

    nlohmann::json j = reusex::core::json_export::to_json_with_defaults(p);
    for (const auto &section : j.at("sections")) {
      std::string sec = section.value("nameEN", std::string{});
      std::map<std::string, nlohmann::json> nested;
      for (const auto &prop : section.at("properties")) {
        std::string col =
            passport_column(sec, prop.value("name", std::string{}));
        if (prop.contains("value")) {
          const nlohmann::json &v = prop["value"];
          row[col] = v.is_string() ? v.get<std::string>() : v.dump();
        } else if (prop.contains("values")) {
          nlohmann::json arr = nlohmann::json::array();
          for (const auto &item : prop["values"])
            arr.push_back(item.value("value", std::string{}));
          row[col] = arr.dump();
        } else if (prop.contains("properties")) {
          nested[col].push_back(prop["properties"]);
        }
      }
      for (auto &[col, arr] : nested)
        row[col] = arr.dump();
    }

    write_row(os, header, row);
  }
}

// --- JSON helpers ------------------------------------------------------------

crow::response error_json_e(crow::status code, const std::string &message) {
  return json_response(code, {{"error", message}});
}

nlohmann::json template_json(const reusex::ProjectDB::ExportTemplateRecord &t) {
  nlohmann::json config = nlohmann::json::parse(t.config_json,
                                                /*cb=*/nullptr,
                                                /*allow_exceptions=*/false);
  if (config.is_discarded())
    config = nlohmann::json::object();
  return nlohmann::json{{"id", t.id},
                        {"name", t.name},
                        {"config", config},
                        {"created_at", t.created_at},
                        {"updated_at", t.updated_at}};
}

} // namespace

void register_export_routes(App &app, EndpointRegistry &reg,
                            reusex::ProjectDB &db) {
  // GET /exports/csv — generate and stream CSV.
  // Optional query param: ?columns=col1,col2,... restricts output columns.
  add_route(
      app, reg,
      {"GET",
       "/exports/csv",
       "Export project elements (components + passports) as CSV",
       true,
       {{200, "CSV bytes"}, {500, "Export failed"}}},
      [&db](const crow::request &req) -> crow::response {
        try {
          // Parse optional column selection from ?columns=... query param.
          std::vector<std::string> select;
          if (const char *raw = req.url_params.get("columns"); raw && *raw) {
            std::istringstream ss(raw);
            std::string col;
            while (std::getline(ss, col, ',')) {
              if (!col.empty())
                select.push_back(col);
            }
          }

          const nlohmann::json blank =
              reusex::core::json_export::generate_blank_template();
          const auto header = build_header(blank, select);

          std::ostringstream oss;
          generate_csv(oss, db, header);

          crow::response res(crow::status::OK);
          res.set_header("Content-Type", "text/csv; charset=utf-8");
          res.set_header("Content-Disposition",
                         "attachment; filename=\"elements.csv\"");
          res.body = oss.str();
          return res;
        } catch (const std::exception &e) {
          reusex::core::error("GET /exports/csv failed: {}", e.what());
          return error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // GET /export-templates — list all stored templates.
  add_route(
      app, reg,
      {"GET",
       "/export-templates",
       "List all named export templates",
       true,
       {{200, "Template list"}}},
      [&db](const crow::request &) -> crow::response {
        try {
          nlohmann::json arr = nlohmann::json::array();
          for (const auto &t : db.list_export_templates())
            arr.push_back(template_json(t));
          return json_response(crow::status::OK,
                               nlohmann::json{{"templates", arr}});
        } catch (const std::exception &e) {
          reusex::core::error("GET /export-templates failed: {}", e.what());
          return error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // POST /export-templates — create a new template.
  add_route(
      app, reg,
      {"POST",
       "/export-templates",
       "Create a named export template",
       true,
       {{201, "Template created"}, {400, "Invalid request body"}}},
      [&db](const crow::request &req) -> crow::response {
        try {
          const auto body = nlohmann::json::parse(req.body, nullptr,
                                                  /*allow_exceptions=*/false);
          if (body.is_discarded() || !body.is_object())
            return error_json_e(crow::status::BAD_REQUEST,
                                "body must be a JSON object");
          if (!body.contains("name") || !body["name"].is_string())
            return error_json_e(crow::status::BAD_REQUEST,
                                "\"name\" (string) is required");

          const std::string name = body["name"].get<std::string>();
          const std::string config_json =
              body.contains("config") ? body["config"].dump() : "{}";

          const auto rec = db.add_export_template(name, config_json);
          reusex::core::info("POST /export-templates: created id={} name={}",
                             rec.id, rec.name);
          return json_response(crow::status::CREATED, template_json(rec));
        } catch (const std::exception &e) {
          reusex::core::error("POST /export-templates failed: {}", e.what());
          return error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // GET /export-templates/<int> — fetch one template by id.
  add_route_dynamic(
      app, reg,
      {"GET",
       "/export-templates/<int>",
       "Fetch a named export template by id",
       true,
       {{200, "Template record"}, {404, "Not found"}}},
      [&db](const crow::request &, crow::response &res, int id) {
        try {
          const auto rec = db.export_template(static_cast<int64_t>(id));
          if (!rec.has_value()) {
            res = error_json_e(crow::status::NOT_FOUND,
                               "export template not found: " +
                                   std::to_string(id));
            res.end();
            return;
          }
          res = json_response(crow::status::OK, template_json(*rec));
          res.end();
        } catch (const std::exception &e) {
          reusex::core::error("GET /export-templates/{} failed: {}", id,
                              e.what());
          res = error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
          res.end();
        }
      });

  // PATCH /export-templates/<int> — update name and/or config.
  add_route_dynamic(
      app, reg,
      {"PATCH",
       "/export-templates/<int>",
       "Update a named export template",
       true,
       {{200, "Updated template"}, {400, "Invalid body"}, {404, "Not found"}}},
      [&db](const crow::request &req, crow::response &res, int id) {
        try {
          const auto existing = db.export_template(static_cast<int64_t>(id));
          if (!existing.has_value()) {
            res = error_json_e(crow::status::NOT_FOUND,
                               "export template not found: " +
                                   std::to_string(id));
            res.end();
            return;
          }

          const auto body = nlohmann::json::parse(req.body, nullptr,
                                                  /*allow_exceptions=*/false);
          if (body.is_discarded() || !body.is_object()) {
            res = error_json_e(crow::status::BAD_REQUEST,
                               "body must be a JSON object");
            res.end();
            return;
          }

          const std::string name =
              body.contains("name") && body["name"].is_string()
                  ? body["name"].get<std::string>()
                  : existing->name;
          const std::string config_json = body.contains("config")
                                              ? body["config"].dump()
                                              : existing->config_json;

          const auto updated = db.update_export_template(
              static_cast<int64_t>(id), name, config_json);
          reusex::core::info("PATCH /export-templates/{}: name={}", id,
                             updated.name);
          res = json_response(crow::status::OK, template_json(updated));
          res.end();
        } catch (const std::exception &e) {
          reusex::core::error("PATCH /export-templates/{} failed: {}", id,
                              e.what());
          res = error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
          res.end();
        }
      });

  // DELETE /export-templates/<int> — remove a template.
  add_route_dynamic(
      app, reg,
      {"DELETE",
       "/export-templates/<int>",
       "Delete a named export template",
       true,
       {{204, "Deleted"}, {404, "Not found"}}},
      [&db](const crow::request &, crow::response &res, int id) {
        try {
          const bool removed =
              db.delete_export_template(static_cast<int64_t>(id));
          if (!removed) {
            res = error_json_e(crow::status::NOT_FOUND,
                               "export template not found: " +
                                   std::to_string(id));
            res.end();
            return;
          }
          reusex::core::info("DELETE /export-templates/{}: removed", id);
          res = crow::response(crow::status::NO_CONTENT);
          res.end();
        } catch (const std::exception &e) {
          reusex::core::error("DELETE /export-templates/{} failed: {}", id,
                              e.what());
          res = error_json_e(crow::status::INTERNAL_SERVER_ERROR, e.what());
          res.end();
        }
      });
}

} // namespace ruxd

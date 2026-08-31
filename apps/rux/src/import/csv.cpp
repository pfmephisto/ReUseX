// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/csv.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/materialepas_json_import.hpp>
#include <reusex/geometry/BuildingComponent.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

/// Parse an RFC-4180 CSV stream into rows of fields (handles quoted fields with
/// embedded commas, quotes, and newlines).
std::vector<std::vector<std::string>> parse_csv(std::istream &is) {
  std::vector<std::vector<std::string>> rows;
  std::vector<std::string> row;
  std::string field;
  bool in_quotes = false;
  bool field_started = false;
  char c;

  auto end_field = [&]() {
    row.push_back(field);
    field.clear();
    field_started = false;
  };
  auto end_row = [&]() {
    end_field();
    rows.push_back(row);
    row.clear();
  };

  while (is.get(c)) {
    if (in_quotes) {
      if (c == '"') {
        if (is.peek() == '"') {
          is.get(c);
          field += '"';
        } else {
          in_quotes = false;
        }
      } else {
        field += c;
      }
    } else {
      if (c == '"' && !field_started) {
        in_quotes = true;
        field_started = true;
      } else if (c == ',') {
        end_field();
      } else if (c == '\r') {
        // swallow; handled by \n
      } else if (c == '\n') {
        end_row();
      } else {
        field += c;
        field_started = true;
      }
    }
  }
  // Flush trailing field/row if the file doesn't end with a newline.
  if (!field.empty() || !row.empty())
    end_row();
  return rows;
}

/// Trim ASCII whitespace from both ends.
std::string trim(const std::string &s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// Lightweight column-addressable view over a parsed row.
struct RowView {
  const std::map<std::string, size_t> *index;
  const std::vector<std::string> *fields;

  bool has(const std::string &col) const {
    auto it = index->find(col);
    return it != index->end() && it->second < fields->size();
  }
  std::string get(const std::string &col) const {
    auto it = index->find(col);
    if (it == index->end() || it->second >= fields->size())
      return "";
    return (*fields)[it->second];
  }
};

/// Apply a component CSV row onto an existing component (mutable fields only).
/// Geometry and type are preserved; type-specific fields are updated to match
/// the component's existing type.
void apply_component_row(reusex::geometry::BuildingComponent &c,
                         const RowView &row) {
  if (row.has("component_name")) {
    std::string name = trim(row.get("component_name"));
    if (!name.empty())
      c.name = name;
  }
  if (row.has("notes"))
    c.notes = row.get("notes");
  if (row.has("confidence")) {
    std::string v = trim(row.get("confidence"));
    if (!v.empty())
      c.confidence = std::stod(v);
  }
  if (row.has("parent_id")) {
    std::string v = trim(row.get("parent_id"));
    c.parent_id = v.empty() ? -1 : std::stoi(v);
  }
  // Provenance link (issue #211). Present only in newer exports; tolerate its
  // absence for CSVs produced before the column existed. A non-empty value
  // overwrites; a blank cell clears the link.
  if (row.has("source_instance"))
    c.source_instance_guid = trim(row.get("source_instance"));

  std::visit(
      [&](auto &&d) {
        using T = std::decay_t<decltype(d)>;
        if constexpr (std::is_same_v<T, reusex::geometry::WindowData>) {
          if (row.has("window_style"))
            d.style = row.get("window_style");
          if (row.has("window_pane_count")) {
            std::string v = trim(row.get("window_pane_count"));
            if (!v.empty())
              d.pane_count = std::stoi(v);
          }
          if (row.has("window_operable")) {
            std::string v = trim(row.get("window_operable"));
            if (!v.empty())
              d.operable = (v == "true" || v == "1");
          }
        } else if constexpr (std::is_same_v<T, reusex::geometry::DoorData>) {
          if (row.has("door_style"))
            d.style = row.get("door_style");
          if (row.has("door_swing"))
            d.swing = row.get("door_swing");
        }
      },
      c.data);
}

/// Overlay scalar passport values from a CSV row onto a passport's JSON.
/// Scalar props (those carrying a "value" key) are updated; array ("values")
/// and nested-object ("properties") props are left as-is (preserved from the
/// existing passport / template).
void overlay_passport_row(json &j, const RowView &row) {
  if (j.contains("sections") && j["sections"].is_array()) {
    for (auto &section : j["sections"]) {
      std::string sec = section.value("nameEN", std::string{});
      if (!section.contains("properties"))
        continue;
      for (auto &prop : section["properties"]) {
        if (!prop.contains("value") || !prop.contains("name"))
          continue;
        std::string col = sec + " / " + prop["name"].get<std::string>();
        if (row.has(col))
          prop["value"] = row.get(col);
      }
    }
  }
  // Metadata columns.
  json &meta = j["metadata"];
  if (row.has("passport_version_number"))
    meta["version number"] = row.get("passport_version_number");
  if (row.has("passport_creation_date"))
    meta["document creation date"] = row.get("passport_creation_date");
  if (row.has("passport_revision_date"))
    meta["document revision date"] = row.get("passport_revision_date");
  if (row.has("passport_version_date"))
    meta["version date"] = row.get("passport_version_date");
}

} // namespace

void setup_subcommand_import_csv(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportCSVOptions>();
  auto *sub = parent.add_subcommand(
      "csv", "Import edited element CSV back into the project");

  sub->footer(R"(
DESCRIPTION:
  Reimports a CSV previously produced by 'rux export csv' after editing. Rows
  are matched by the immutable 'id' column (component guid / passport document
  guid):

    - Components: updates mutable fields (component_name, notes, confidence,
      parent_id, and type-specific window/door fields) on the existing
      component. Geometry and type are NOT changed. Rows whose guid does not
      exist are skipped (components can't be created from CSV — no geometry).
    - Passports: scalar fields are written back; a passport whose guid does not
      exist yet is created. Array cells (e.g. materials) and nested cells (e.g.
      dangerous substances) are preserved from the database and NOT updated.

EXAMPLES:
  rux import csv elements.csv                 # Reimport edits
  rux -p scan.rux import csv edited.csv       # Custom project

NOTES:
  - Do not edit the 'id' column; it is the match key
  - Rename components via 'component_name', not 'id'
  - Only scalar passport fields round-trip; array/nested fields are read-only
)");

  sub->add_option("input", opt->input_path, "Input CSV file (.csv)")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_option("--project-id", opt->project_id,
                  "Project id for passports created by this import (optional)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling import csv subcommand");
    return run_subcommand_import_csv(*opt, *global_opt);
  });
}

int run_subcommand_import_csv(SubcommandImportCSVOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    // 1. Parse CSV.
    std::ifstream ifs(opt.input_path);
    if (!ifs.is_open()) {
      spdlog::error("Failed to open input file: {}", opt.input_path.string());
      return RuxError::IO;
    }
    auto rows = parse_csv(ifs);
    if (rows.empty()) {
      spdlog::warn("CSV file is empty");
      return RuxError::SUCCESS;
    }

    // 2. Build header index.
    std::map<std::string, size_t> col_index;
    for (size_t i = 0; i < rows[0].size(); ++i)
      col_index[rows[0][i]] = i;
    if (!col_index.count("kind") || !col_index.count("id")) {
      spdlog::error("CSV missing required 'kind' and 'id' columns");
      return RuxError::INVALID_ARGUMENT;
    }

    // 3. Open database read/write.
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path, /*readOnly=*/false);

    // Pre-load existing components indexed by guid.
    std::map<std::string, reusex::geometry::BuildingComponent> by_guid;
    for (const auto &name : db.list_building_components()) {
      auto c = db.building_component(name);
      if (!c.guid.empty())
        by_guid.emplace(c.guid, std::move(c));
    }

    size_t comp_updated = 0, comp_skipped = 0, pass_imported = 0;

    for (size_t r = 1; r < rows.size(); ++r) {
      RowView row{&col_index, &rows[r]};
      std::string kind = trim(row.get("kind"));
      std::string id = trim(row.get("id"));
      if (kind.empty() && id.empty())
        continue; // blank line

      if (kind == "component") {
        if (id.empty()) {
          spdlog::warn("Row {}: component with empty id, skipping", r + 1);
          ++comp_skipped;
          continue;
        }
        auto it = by_guid.find(id);
        if (it == by_guid.end()) {
          spdlog::warn("Row {}: no component with guid '{}' (can't create from "
                       "CSV), skipping",
                       r + 1, id);
          ++comp_skipped;
          continue;
        }
        reusex::geometry::BuildingComponent c = it->second;
        apply_component_row(c, row);
        db.update_building_component_by_guid(c);
        ++comp_updated;
      } else if (kind == "passport") {
        if (id.empty()) {
          spdlog::warn("Row {}: passport with empty id, skipping", r + 1);
          continue;
        }
        // Base JSON from the existing passport, or a blank template for a new
        // one, so array/nested fields are preserved.
        json base;
        std::string row_id = id; // material_passports.id column
        try {
          auto existing = db.material_passport(id);
          base = reusex::core::json_export::to_json(existing);
          // Preserve a sensor-frame link: the row id is the node id when the
          // passport was matched to a frame, otherwise the document guid.
          if (auto node = db.passport_linked_node_id(id))
            row_id = std::to_string(*node);
        } catch (const std::exception &) {
          reusex::core::MaterialPassport blank;
          blank.metadata.document_guid = id;
          base = reusex::core::json_export::to_json(blank);
        }
        overlay_passport_row(base, row);
        base["metadata"]["document guid"] = id; // never let an edit change it

        auto passport = reusex::core::json_import::from_json(base);
        db.add_material_passport(passport, opt.project_id, row_id);
        ++pass_imported;
      } else {
        spdlog::warn("Row {}: unknown kind '{}', skipping", r + 1, kind);
      }
    }

    spdlog::info("Imported CSV: {} component(s) updated, {} skipped, {} "
                 "passport(s) written",
                 comp_updated, comp_skipped, pass_imported);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("CSV import failed: {}", e.what());
    return RuxError::IO;
  }
}

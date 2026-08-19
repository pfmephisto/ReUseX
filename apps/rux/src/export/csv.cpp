// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/csv.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/geometry/BuildingComponent.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

// Fixed columns describing a geometry instance (building component). These stay
// blank on passport rows. The 'id' column holds the immutable guid (the match
// key for reimport); 'component_name' is the editable, human-facing name.
const std::vector<std::string> k_component_cols = {
    "component_name",  "component_type",    "confidence",
    "parent_id",       "notes",             "window_style",
    "window_pane_count", "window_operable", "door_style",
    "door_swing"};

// Fixed metadata columns for a material passport. Blank on component rows.
// 'linked_instance' shows the instance a passport was generated from, e.g.
// "instances#7" (see 'rux create materials').
const std::vector<std::string> k_passport_meta_cols = {
    "linked_instance",         "passport_version_number",
    "passport_creation_date",  "passport_revision_date",
    "passport_version_date"};

/// Escape a field for RFC-4180 CSV output.
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

/// Column name for a passport property inside a named section.
std::string passport_column(const std::string &section_en,
                            const std::string &prop_name) {
  return section_en + " / " + prop_name;
}

/// Ordered, de-duplicated list of passport property columns derived from a
/// template's "sections" array (as produced by json_export::to_json).
std::vector<std::string> passport_property_columns(const json &sections) {
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

/// Write a single CSV record from a column->value map in header order.
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

} // namespace

void setup_subcommand_export_csv(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportCSVOptions>();
  auto *sub = parent.add_subcommand(
      "csv", "Export all elements (components + passports) to CSV");

  sub->footer(R"(
DESCRIPTION:
  Exports every element in a ReUseX project database to a single wide CSV file,
  combining geometry instances (building components: windows/doors/walls) and
  resources (material passports / "materialepas"). A 'kind' column discriminates
  'component' from 'passport'; the columns are the union of both sources, so
  component rows leave passport columns blank and vice-versa.

  Building components are exported with their properties only (type, confidence,
  parent_id, notes, and type-specific window/door fields). Material passports are
  exported with the full Danish standard template (~75 fields, blank where unset).

EXAMPLES:
  rux export csv                     # Export to elements.csv
  rux export csv -o out.csv          # Custom output file
  rux -p scan.rux export csv         # Custom project path

NOTES:
  - Default output: elements.csv in current directory
  - The 'id' column is an immutable GUID (component guid / passport document
    guid). Keep it intact to reimport edits with 'rux import csv'.
  - Edit 'component_name' (not 'id') to rename a component safely
  - Nested passport fields (e.g. dangerous substances) render as JSON in a cell
  - Fields containing commas/quotes/newlines are quoted per RFC 4180
  - Warns if the project has no elements (not an error)
)");

  sub->add_option("-o,--output", opt->output_path,
                  "Output CSV file path (default: elements.csv)")
      ->default_val(opt->output_path);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export csv subcommand");
    return run_subcommand_export_csv(*opt, *global_opt);
  });
}

int run_subcommand_export_csv(SubcommandExportCSVOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path, /*readOnly=*/true);

    // Derive the passport property columns from a blank standard template so the
    // schema is stable regardless of how many passports (if any) exist.
    json blank = reusex::core::json_export::generate_blank_template();
    std::vector<std::string> passport_cols =
        passport_property_columns(blank.at("sections"));

    // Assemble the full header: leading + component + passport meta + passport
    // property columns.
    std::vector<std::string> header = {"kind", "id"};
    header.insert(header.end(), k_component_cols.begin(),
                  k_component_cols.end());
    header.insert(header.end(), k_passport_meta_cols.begin(),
                  k_passport_meta_cols.end());
    header.insert(header.end(), passport_cols.begin(), passport_cols.end());

    std::ofstream ofs(opt.output_path);
    if (!ofs.is_open()) {
      spdlog::error("Failed to open output file: {}", opt.output_path.string());
      return RuxError::IO;
    }

    // Header row.
    for (size_t i = 0; i < header.size(); ++i) {
      if (i)
        ofs << ',';
      ofs << csv_escape(header[i]);
    }
    ofs << '\n';

    // --- Geometry instances (building components) ---
    auto component_names = db.list_building_components();
    for (const auto &name : component_names) {
      auto c = db.building_component(name);
      std::map<std::string, std::string> row;
      row["kind"] = "component";
      row["id"] = c.guid;
      row["component_name"] = c.name;
      row["component_type"] =
          std::string(reusex::geometry::to_string(c.type));
      if (c.confidence >= 0.0)
        row["confidence"] = std::to_string(c.confidence);
      if (c.parent_id >= 0)
        row["parent_id"] = std::to_string(c.parent_id);
      row["notes"] = c.notes;

      std::visit(
          [&](auto &&d) {
            using T = std::decay_t<decltype(d)>;
            if constexpr (std::is_same_v<T, reusex::geometry::WindowData>) {
              row["window_style"] = d.style;
              row["window_pane_count"] = std::to_string(d.pane_count);
              row["window_operable"] = d.operable ? "true" : "false";
            } else if constexpr (std::is_same_v<
                                     T, reusex::geometry::DoorData>) {
              row["door_style"] = d.style;
              row["door_swing"] = d.swing;
            }
          },
          c.data);

      write_row(ofs, header, row);
    }

    // --- Resources (material passports) ---
    // Reverse map material guid -> "cloud#instance" so each passport row can
    // show which instance it was generated from.
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

      json j = reusex::core::json_export::to_json_with_defaults(p);
      for (const auto &section : j.at("sections")) {
        std::string sec = section.value("nameEN", std::string{});
        // Accumulate nested/duplicate property entries into a JSON array per
        // column so repeated fields collapse into a single cell.
        std::map<std::string, json> nested;
        for (const auto &prop : section.at("properties")) {
          std::string col =
              passport_column(sec, prop.value("name", std::string{}));
          if (prop.contains("value")) {
            const json &v = prop["value"];
            row[col] = v.is_string() ? v.get<std::string>() : v.dump();
          } else if (prop.contains("values")) {
            // StringArray / EnumArray: render as a compact JSON array of the
            // value strings, e.g. ["brick","steel"].
            json arr = json::array();
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

      write_row(ofs, header, row);
    }

    if (!ofs) {
      spdlog::error("Failed to write output file: {}",
                    opt.output_path.string());
      return RuxError::IO;
    }

    size_t total = component_names.size() + passports.size();
    if (total == 0)
      spdlog::warn("No elements found in database (wrote header only)");
    spdlog::info("Exported {} element(s) ({} component(s), {} passport(s)) to {}",
                 total, component_names.size(), passports.size(),
                 opt.output_path.string());
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("Export failed: {}", e.what());
    return RuxError::IO;
  }
}

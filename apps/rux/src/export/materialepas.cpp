// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/materialepas.hpp"
#include "global-params.hpp"

#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/ProjectDB.hpp>

#include <fstream>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

void setup_subcommand_export_materialepas(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportMaterialepasOptions>();
  auto *sub = parent.add_subcommand(
      "materialepas",
      "Export material passports to JSON");

  sub->footer(R"(
DESCRIPTION:
  Exports all material passports from a ReUseX project database to JSON
  following the Danish "Materialepas for genbrugte byggevarer" interchange
  format. Enables data sharing, archival, and integration with external
  building reuse and circular economy systems.

EXAMPLES:
  rux export materialepas              # Export to materialepas.json
  rux export materialepas -o out.json  # Custom output file
  rux -p scan.rux export materialepas  # Custom project path

WORKFLOW:
  1. rux create material               # Create passport template
  2. rux import materialepas data.json # Import passport data
  3. # Enrich with component data
  4. rux export materialepas -o final.json  # Export for sharing

NOTES:
  - Default output: materialepas.json in current directory
  - Exports all passports stored in project database
  - JSON format compatible with Danish building reuse standard
  - Warns if no passports found (not an error)
  - Use 'rux get passports' to list available passports first
)");

  sub->add_option("-o,--output", opt->output_path,
                  "Output JSON file path (default: materialepas.json)")
      ->default_val(opt->output_path);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export materialepas subcommand");
    return run_subcommand_export_materialepas(*opt, *global_opt);
  });
}

int run_subcommand_export_materialepas(
    SubcommandExportMaterialepasOptions const &opt, const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    // 1. Open database read-only
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path, /*readOnly=*/true);

    // 2. Retrieve all material passports
    auto passports = db.all_material_passports();
    if (passports.empty()) {
      spdlog::warn("No material passports found in database");
      return RuxError::SUCCESS;
    }
    spdlog::info("Found {} material passport(s)", passports.size());

    // 3. Serialize to JSON with defaults (full ~75-field interchange format)
    nlohmann::json::array_t passports_json;
    for (const auto &passport : passports) {
      passports_json.push_back(
          reusex::core::json_export::to_json_with_defaults(passport));
    }
    nlohmann::json json_data = passports_json;
    std::string json_str = json_data.dump(4);

    // 4. Write to output file
    std::ofstream ofs(opt.output_path);
    if (!ofs.is_open()) {
      spdlog::error("Failed to open output file: {}",
                     opt.output_path.string());
      return RuxError::IO;
    }

    ofs << json_str;
    if (!ofs) {
      spdlog::error("Failed to write output file: {}",
                     opt.output_path.string());
      return RuxError::IO;
    }

    spdlog::info("Exported {} passport(s) to {}", passports.size(),
                 opt.output_path.string());
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("Export failed: {}", e.what());
    return RuxError::IO;
  }
}

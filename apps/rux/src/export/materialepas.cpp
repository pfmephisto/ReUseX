// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/materialepas.hpp"
#include "global-params.hpp"

#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/project_db.hpp>

#include <fstream>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

void setup_subcommand_export_materialepas(CLI::App &parent) {
  auto opt = std::make_shared<SubcommandExportMaterialepasOptions>();
  auto *sub = parent.add_subcommand(
      "materialepas",
      "Export material passports from a project database to JSON.\n\n"
      "Reads all material passports stored in a ReUseX project database\n"
      "and writes them as a JSON file following the Danish\n"
      "\"Materialepas for genbrugte byggevarer\" interchange format.\n\n"
      "Examples:\n"
      "  rux export materialepas project.db\n"
      "  rux export materialepas project.db -o passports.json");

  sub->add_option("input", opt->input_path, "Input project database (.db)")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-o,--output", opt->output_path,
                  "Output JSON file path (default: materialepas.json)")
      ->default_val(opt->output_path);

  sub->callback([opt]() {
    spdlog::trace("calling export materialepas subcommand");
    return run_subcommand_export_materialepas(*opt);
  });
}

int run_subcommand_export_materialepas(
    SubcommandExportMaterialepasOptions const &opt) {
  try {
    // 1. Open database read-only
    spdlog::info("Opening project database: {}", opt.input_path.string());
    ReUseX::ProjectDB db(opt.input_path, /*readOnly=*/true);

    // 2. Retrieve all material passports
    auto passports = db.getAllMaterialPassports();
    if (passports.empty()) {
      spdlog::warn("No material passports found in database");
      return RuxError::SUCCESS;
    }
    spdlog::info("Found {} material passport(s)", passports.size());

    // 3. Serialize to JSON
    std::string json_str =
        ReUseX::core::json_export::to_json_string(passports);

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

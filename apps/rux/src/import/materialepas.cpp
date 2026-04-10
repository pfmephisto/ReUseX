// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/materialepas.hpp"
#include "global-params.hpp"

#include <reusex/core/materialepas_json_import.hpp>
#include <reusex/core/ProjectDB.hpp>

#include <fstream>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

void setup_subcommand_import_materialepas(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportMaterialepasOptions>();
  auto *sub = parent.add_subcommand(
      "materialepas",
      "Import material passports from a JSON file into a project database.\n\n"
      "Reads a JSON file following the Danish\n"
      "\"Materialepas for genbrugte byggevarer\" interchange format\n"
      "and stores the material passports in a ReUseX project database.\n\n"
      "Examples:\n"
      "  rux import materialepas materials.json\n"
      "  rux import materialepas materials.json --project-id my_project");

  sub->add_option("input", opt->input_path, "Input JSON file (.json)")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--project-id", opt->project_id,
                  "Project identifier (optional, passport stored without project if omitted)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling import materialepas subcommand");
    return run_subcommand_import_materialepas(*opt, *global_opt);
  });
}

int run_subcommand_import_materialepas(
    SubcommandImportMaterialepasOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;

  // 1. Read JSON file
  spdlog::info("Reading JSON file: {}", opt.input_path.string());
  std::ifstream ifs(opt.input_path);
  if (!ifs.is_open()) {
    spdlog::error("Failed to open input file: {}", opt.input_path.string());
    return RuxError::IO;
  }

  std::string json_str((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
  ifs.close();

  // 2. Parse JSON → MaterialPassport(s)
  std::vector<ReUseX::core::MaterialPassport> passports;
  try {
    passports = ReUseX::core::json_import::from_json_string(json_str);
  } catch (const std::exception &e) {
    spdlog::error("Failed to parse JSON: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  }

  if (passports.empty()) {
    spdlog::warn("No material passports found in JSON file");
    return RuxError::SUCCESS;
  }
  spdlog::info("Parsed {} material passport(s)", passports.size());

  // 3. Open database and store passports
  try {
    spdlog::info("Opening project database: {}", project_path.string());
    ReUseX::ProjectDB db(project_path, /*readOnly=*/false);

    for (const auto &passport : passports) {
      db.add_material_passport(passport, opt.project_id);
    }
  } catch (const std::exception &e) {
    spdlog::error("Database import failed: {}", e.what());
    return RuxError::IO;
  }

  spdlog::info("Imported {} passport(s) into database with project_id='{}'",
               passports.size(), opt.project_id);
  return RuxError::SUCCESS;
}

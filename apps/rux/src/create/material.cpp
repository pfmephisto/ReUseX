// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/material.hpp"
#include "global-params.hpp"

#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/materialepas_json_import.hpp>
#include <reusex/core/ProjectDB.hpp>

#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>

void setup_subcommand_create_material(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateMaterialOptions>();
  auto *sub = parent.add_subcommand(
      "material",
      "Create a new blank material passport.\n\n"
      "Generates a new material passport with all fields present but empty,\n"
      "ready for manual or programmatic population. Metadata fields (GUID,\n"
      "dates, version) are auto-generated.\n\n"
      "Usage modes:\n"
      "  1. Output JSON to stdout (default):\n"
      "     rux create material\n"
      "  2. Save to project database:\n"
      "     rux create material\n"
      "  3. Save to both database and file:\n"
      "     rux create material -o template.json\n\n"
      "Examples:\n"
      "  rux create material                        # Output to stdout\n"
      "  rux create material > blank.json           # Redirect to file\n"
      "  rux create material                        # Save to database\n"
      "  rux create material --guid my-guid         # Custom GUID\n"
      "  rux create material -o out.json            # Save to both");

  sub->add_option("--guid", opt->guid,
                  "Custom GUID for the material passport (default: auto-generated)");

  sub->add_option("-o,--output", opt->output_file,
                  "Output JSON file path (optional)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create material subcommand");
    return run_subcommand_create_material(*opt, *global_opt);
  });
}

int run_subcommand_create_material(SubcommandCreateMaterialOptions const &opt, const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;

    // 1. Generate blank template
    nlohmann::json template_json = ReUseX::core::json_export::generate_blank_template();

    // 2. Override GUID if custom one was provided
    if (!opt.guid.empty()) {
      template_json["metadata"]["document guid"] = opt.guid;
    }

    std::string generated_guid = template_json["metadata"]["document guid"];

    // 3. Save to project database
    // Parse JSON back to MaterialPassport
    auto passport = ReUseX::core::json_import::from_json(template_json);

    // Open database (will create if doesn't exist) and save
    ReUseX::ProjectDB db(project_path, /*readOnly=*/false);
    db.add_material_passport(passport, "");

    spdlog::info("Created material passport '{}' in project: {}",
                 generated_guid, project_path.string());

    // 4. Output JSON (to file or stdout) if requested
    if (!opt.output_file.empty()) {
      std::string json_str = template_json.dump(4);
      // Write to file
      std::ofstream ofs(opt.output_file);
      if (!ofs.is_open()) {
        spdlog::error("Failed to open output file: {}", opt.output_file.string());
        return RuxError::IO;
      }
      ofs << json_str << std::endl;
      if (!ofs) {
        spdlog::error("Failed to write output file: {}", opt.output_file.string());
        return RuxError::IO;
      }
      spdlog::info("Wrote template to: {}", opt.output_file.string());
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Material passport creation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

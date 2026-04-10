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

void setup_subcommand_create_material(CLI::App &parent) {
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
      "     rux create material -p project.rux\n"
      "  3. Save to both database and file:\n"
      "     rux create material -p project.rux -o template.json\n\n"
      "Examples:\n"
      "  rux create material                        # Output to stdout\n"
      "  rux create material > blank.json           # Redirect to file\n"
      "  rux create material -p project.rux         # Save to database\n"
      "  rux create material --guid my-guid         # Custom GUID\n"
      "  rux create material -p project.rux -o out.json  # Save to both");

  sub->add_option("--guid", opt->guid,
                  "Custom GUID for the material passport (default: auto-generated)");

  sub->add_option("-p,--project", opt->project,
                  "Project database to save passport to (optional)");

  sub->add_option("-o,--output", opt->output_file,
                  "Output JSON file path (optional, use with --project)");

  sub->callback([opt]() {
    spdlog::trace("calling create material subcommand");
    return run_subcommand_create_material(*opt);
  });
}

int run_subcommand_create_material(SubcommandCreateMaterialOptions const &opt) {
  try {
    // 1. Generate blank template
    nlohmann::json template_json = ReUseX::core::json_export::generate_blank_template();

    // 2. Override GUID if custom one was provided
    if (!opt.guid.empty()) {
      template_json["metadata"]["document guid"] = opt.guid;
    }

    std::string generated_guid = template_json["metadata"]["document guid"];

    // 3. Save to project database if specified
    if (!opt.project.empty()) {
      // Parse JSON back to MaterialPassport
      auto passport = ReUseX::core::json_import::from_json(template_json);

      // Open database (will create if doesn't exist) and save
      ReUseX::ProjectDB db(opt.project, /*readOnly=*/false);
      db.add_material_passport(passport, "");

      spdlog::info("Created material passport '{}' in project: {}",
                   generated_guid, opt.project.string());
    }

    // 4. Output JSON (to file or stdout)
    std::string json_str = template_json.dump(4);

    if (!opt.output_file.empty()) {
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
    } else if (opt.project.empty()) {
      // No project specified = output to stdout (default behavior)
      std::cout << json_str << std::endl;
    }
    // else: project specified but no output file = silent (just saved to DB)

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Material passport creation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

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
      "Create blank material passport");

  sub->footer(R"(
DESCRIPTION:
  Generates a new material passport following the Danish "Materialepas for
  genbrugte byggevarer" schema. All fields are present but empty, ready
  for manual or programmatic population. Metadata fields (GUID, dates,
  version) are auto-generated. Output to stdout, file, or database.

EXAMPLES:
  rux create material                  # Output JSON to stdout
  rux create material > blank.json     # Redirect to file
  rux create material -o template.json # Save to file and database
  rux create material --guid my-guid   # Custom GUID

WORKFLOW:
  1. rux create material > base.json   # Create template
  2. # Edit base.json with material data
  3. rux import materialepas base.json # Import to database
  4. rux export materialepas -o out.json  # Export for sharing

NOTES:
  - Default: outputs to stdout for piping or redirection
  - Use -o/--output to save JSON file alongside database entry
  - GUID auto-generated (UUID v4) unless --guid specified
  - Timestamps auto-populated with current date
  - Schema version set to latest Danish standard
  - Saved to project database by default
)");

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
    nlohmann::json template_json = reusex::core::json_export::generate_blank_template();

    // 2. Override GUID if custom one was provided
    if (!opt.guid.empty()) {
      template_json["metadata"]["document guid"] = opt.guid;
    }

    std::string generated_guid = template_json["metadata"]["document guid"];

    // 3. Save to project database
    // Parse JSON back to MaterialPassport
    auto passport = reusex::core::json_import::from_json(template_json);

    // Open database (will create if doesn't exist) and save
    reusex::ProjectDB db(project_path, /*readOnly=*/false);
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

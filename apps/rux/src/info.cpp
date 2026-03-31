// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "info.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <locale>

using json = nlohmann::json;

namespace {

// Format terminal output with detailed information
void format_terminal_output(const ReUseX::ProjectDB::ProjectSummary &summary) {
  // Enable thousand separators for numbers
  std::locale loc("");
  std::cout.imbue(loc);

  std::cout << fmt::format("Project: {}\n", summary.path.string());
  std::cout << fmt::format("Schema Version: {}\n\n", summary.schema_version);

  // Sensor frames section
  if (summary.sensor_frames.total_count > 0) {
    if (summary.sensor_frames.width > 0 && summary.sensor_frames.height > 0) {
      std::cout << fmt::format("Sensor Frames: {} frames ({}x{})\n",
                               summary.sensor_frames.total_count,
                               summary.sensor_frames.width,
                               summary.sensor_frames.height);
    } else {
      std::cout << fmt::format("Sensor Frames: {} frames\n",
                               summary.sensor_frames.total_count);
    }
    std::cout << fmt::format("  Segmented: {} frames\n", summary.sensor_frames.segmented_count);
  } else {
    std::cout << "Sensor Frames: 0 frames\n";
  }
  std::cout << "\n";

  // Point clouds section
  std::cout << fmt::format("Point Clouds: {}\n", summary.clouds.size());
  for (const auto &cloud : summary.clouds) {
    std::cout << fmt::format("  {}\n", cloud.name);
    std::cout << fmt::format("    Type: {}\n", cloud.type);
    std::cout << fmt::format("    Points: {}\n", cloud.point_count);

    std::string org = cloud.organized ? "organized" : "unorganized";
    std::cout << fmt::format("    Dimensions: {}x{} ({})\n",
                             cloud.width, cloud.height, org);

    // Show labels for Label clouds
    if (cloud.type == "Label" && !cloud.labels.empty()) {
      std::string label_list;
      bool first = true;
      for (const auto &[id, name] : cloud.labels) {
        if (!first) label_list += ", ";
        label_list += name;
        first = false;
      }
      std::cout << fmt::format("    Labels: {} ({} classes)\n",
                               label_list, cloud.labels.size());
    }
    std::cout << "\n";
  }

  // Meshes section
  std::cout << fmt::format("Meshes: {}\n", summary.meshes.size());
  for (const auto &mesh : summary.meshes) {
    std::cout << fmt::format("  {}\n", mesh.name);
    std::cout << fmt::format("    Vertices: {}\n", mesh.vertex_count);
    std::cout << fmt::format("    Faces: {}\n\n", mesh.face_count);
  }

  // Material passports section
  std::cout << fmt::format("Material Passports: {} documents\n",
                           summary.material_passport_count);
}

// Format JSON output
void format_json_output(const ReUseX::ProjectDB::ProjectSummary &summary) {
  json j;

  j["project_path"] = summary.path.string();
  j["schema_version"] = summary.schema_version;

  // Sensor frames
  j["sensor_frames"] = {
    {"total", summary.sensor_frames.total_count},
    {"width", summary.sensor_frames.width},
    {"height", summary.sensor_frames.height},
    {"segmented", summary.sensor_frames.segmented_count}
  };

  // Point clouds
  j["point_clouds"] = json::array();
  for (const auto &cloud : summary.clouds) {
    json cloud_json = {
      {"name", cloud.name},
      {"type", cloud.type},
      {"point_count", cloud.point_count},
      {"width", cloud.width},
      {"height", cloud.height},
      {"organized", cloud.organized}
    };

    // Add labels for Label clouds
    if (cloud.type == "Label" && !cloud.labels.empty()) {
      json labels_json;
      for (const auto &[id, name] : cloud.labels) {
        labels_json[std::to_string(id)] = name;
      }
      cloud_json["labels"] = labels_json;
    }

    j["point_clouds"].push_back(cloud_json);
  }

  // Meshes
  j["meshes"] = json::array();
  for (const auto &mesh : summary.meshes) {
    j["meshes"].push_back({
      {"name", mesh.name},
      {"vertex_count", mesh.vertex_count},
      {"face_count", mesh.face_count}
    });
  }

  // Material passports
  j["material_passports"] = {
    {"count", summary.material_passport_count}
  };

  // Output pretty-printed JSON to stdout
  std::cout << j.dump(2) << std::endl;
}

} // anonymous namespace

void setup_subcommand_info(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "info", "Display summary information about a project database");

  static SubcommandInfoOptions opt;

  sub->add_option("project", opt.project, "Path to project database file (.rux)")
      ->default_val(GlobalParams::project_db);

  sub->add_flag("-j,--json", opt.json_output, "Output in JSON format")
      ->default_val(false);

  sub->callback([&opt]() {
    spdlog::trace("Running info subcommand");
    return run_subcommand_info(opt);
  });
}

int run_subcommand_info(SubcommandInfoOptions const &opt) {
  try {
    // Open project database in read-only mode
    ReUseX::ProjectDB db(opt.project, /* readOnly */ true);

    // Get project summary
    auto summary = db.project_summary();

    // Format output
    if (opt.json_output) {
      format_json_output(summary);
    } else {
      format_terminal_output(summary);
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to read project info: {}", e.what());
    return RuxError::IO;
  }
}

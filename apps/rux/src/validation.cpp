// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "validation.hpp"
#include <fmt/format.h>
#include <fmt/ranges.h>

namespace rux::validation {

ValidationResult validate_clouds_prerequisites(const ReUseX::ProjectDB &db) {
  auto frame_ids = db.sensor_frame_ids();
  if (frame_ids.empty()) {
    return ValidationResult::error(
        "No sensor frames found in database",
        "Run 'rux import rtabmap scan.db --project project.rux' to import "
        "sensor data",
        {"sensor_frames"});
  }
  return ValidationResult::ok();
}

ValidationResult validate_planes_prerequisites(const ReUseX::ProjectDB &db) {
  std::vector<std::string> missing;

  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");
  if (!db.has_point_cloud("normals"))
    missing.push_back("normals");

  if (!missing.empty()) {
    return ValidationResult::error(
        fmt::format("Missing required point clouds: {}",
                    fmt::join(missing, ", ")),
        "Run 'rux create clouds project.rux' to generate point clouds from "
        "sensor frames",
        missing);
  }

  return ValidationResult::ok();
}

ValidationResult validate_rooms_prerequisites(const ReUseX::ProjectDB &db) {
  std::vector<std::string> missing;

  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");
  if (!db.has_point_cloud("planes"))
    missing.push_back("planes");
  if (!db.has_point_cloud("plane_centroids"))
    missing.push_back("plane_centroids");
  if (!db.has_point_cloud("plane_normals"))
    missing.push_back("plane_normals");

  if (!missing.empty()) {
    // Provide progressive guidance based on what's missing
    std::string resolution;
    if (!db.has_point_cloud("cloud")) {
      resolution = "Run 'rux create clouds project.rux' first, then 'rux "
                   "create planes project.rux'";
    } else if (!db.has_point_cloud("planes")) {
      resolution =
          "Run 'rux create planes project.rux' to segment planar surfaces";
    } else {
      resolution = "Run 'rux create planes project.rux' to generate plane data";
    }

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  return ValidationResult::ok();
}

ValidationResult validate_mesh_prerequisites(const ReUseX::ProjectDB &db) {
  std::vector<std::string> missing;

  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");
  if (!db.has_point_cloud("normals"))
    missing.push_back("normals");
  if (!db.has_point_cloud("rooms"))
    missing.push_back("rooms");
  if (!db.has_point_cloud("planes"))
    missing.push_back("planes");
  if (!db.has_point_cloud("plane_centroids"))
    missing.push_back("plane_centroids");
  if (!db.has_point_cloud("plane_normals"))
    missing.push_back("plane_normals");

  if (!missing.empty()) {
    // Build progressive resolution hint
    std::vector<std::string> steps;

    if (!db.has_point_cloud("cloud")) {
      steps.push_back("rux create clouds project.rux");
    }
    if (!db.has_point_cloud("planes")) {
      steps.push_back("rux create planes project.rux");
    }
    if (!db.has_point_cloud("rooms")) {
      steps.push_back("rux create rooms project.rux");
    }

    std::string resolution =
        fmt::format("Run the following commands in order:\n    {}",
                    fmt::join(steps, "\n    "));

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  // Size validation after existence check
  // This addresses the TODO comment in mesh.cpp lines 101-107
  try {
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto normals = db.point_cloud_normal("normals");
    auto rooms = db.point_cloud_label("rooms");
    auto planes = db.point_cloud_label("planes");

    // Check all clouds have same size
    if (cloud->size() != normals->size() || cloud->size() != rooms->size() ||
        cloud->size() != planes->size()) {
      return ValidationResult::error(
          fmt::format("Point cloud size mismatch: cloud={}, normals={}, "
                      "rooms={}, planes={}",
                      cloud->size(), normals->size(), rooms->size(),
                      planes->size()),
          "Data corruption detected - regenerate point clouds with 'rux "
          "create clouds project.rux'",
          {});
    }
  } catch (const std::exception &e) {
    return ValidationResult::error(
        fmt::format("Failed to validate data: {}", e.what()),
        "Check database integrity with 'rux info project.rux'", {});
  }

  return ValidationResult::ok();
}

ValidationResult validate_texture_prerequisites(const ReUseX::ProjectDB &db) {
  std::vector<std::string> missing;

  if (!db.has_mesh("mesh"))
    missing.push_back("mesh");

  auto frame_ids = db.sensor_frame_ids();
  if (frame_ids.empty())
    missing.push_back("sensor_frames");

  if (!missing.empty()) {
    std::string resolution;

    if (!db.has_mesh("mesh")) {
      resolution = "Run the mesh generation pipeline: clouds → planes → rooms "
                   "→ mesh";
    } else if (frame_ids.empty()) {
      resolution = "Run 'rux import rtabmap scan.db --project project.rux' to "
                   "import sensor data";
    }

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  return ValidationResult::ok();
}

ValidationResult validate_project_prerequisites(const ReUseX::ProjectDB &db) {
  std::vector<std::string> missing;

  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");

  // Check for at least one segmentation image
  auto frame_ids = db.sensor_frame_ids();
  bool has_segmentation = false;
  for (const auto &id : frame_ids) {
    if (db.has_segmentation_image(id)) {
      has_segmentation = true;
      break;
    }
  }

  if (!has_segmentation)
    missing.push_back("segmentation_images");

  if (!missing.empty()) {
    std::string resolution;

    if (!db.has_point_cloud("cloud")) {
      resolution = "Run 'rux create clouds project.rux' first, then 'rux "
                   "annotate project.rux --net model.engine'";
    } else if (!has_segmentation) {
      resolution =
          "Run 'rux annotate project.rux --net model.engine' to generate "
          "semantic segmentation";
    }

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  return ValidationResult::ok();
}

ValidationResult validate_annotate_prerequisites(const ReUseX::ProjectDB &db) {
  auto frame_ids = db.sensor_frame_ids();
  if (frame_ids.empty()) {
    return ValidationResult::error(
        "No sensor frames found in database",
        "Run 'rux import rtabmap scan.db --project project.rux' to import "
        "sensor data",
        {"sensor_frames"});
  }
  return ValidationResult::ok();
}

ValidationResult
validate_instances_prerequisites(const ReUseX::ProjectDB &db,
                                 const std::string &semantic_cloud_name) {

  std::vector<std::string> missing;

  // Check cloud exists
  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");

  // Check semantic labels exist
  if (!db.has_point_cloud(semantic_cloud_name))
    missing.push_back(semantic_cloud_name);

  if (!missing.empty()) {
    std::string resolution;
    if (!db.has_point_cloud("cloud")) {
      resolution = "Run 'rux create clouds project.rux' first";
    } else {
      auto available = db.list_point_clouds();
      resolution = fmt::format("Semantic cloud '{}' not found. Available: {}",
                               semantic_cloud_name, fmt::join(available, ", "));
    }

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  // Size validation
  try {
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto semantic = db.point_cloud_label(semantic_cloud_name);

    if (cloud->size() != semantic->size()) {
      return ValidationResult::error(
          fmt::format("Size mismatch: cloud={}, {}={}", cloud->size(),
                      semantic_cloud_name, semantic->size()),
          "Regenerate point clouds with 'rux create clouds project.rux'", {});
    }
  } catch (const std::exception &e) {
    return ValidationResult::error(
        fmt::format("Failed to load data: {}", e.what()),
        "Check database with 'rux info project.rux'", {});
  }

  return ValidationResult::ok();
}
ValidationResult
validate_window_prerequisites(const ReUseX::ProjectDB &db,
                              const std::string &semantic_cloud_name) {
  std::vector<std::string> missing;

  // Check cloud exists
  if (!db.has_point_cloud("cloud"))
    missing.push_back("cloud");

  // Check semantic labels exist
  if (!db.has_point_cloud(semantic_cloud_name))
    missing.push_back(semantic_cloud_name);

  if (!db.has_point_cloud("instances"))
    missing.push_back("instances");

  if (!db.has_mesh("mesh"))
    missing.push_back("mesh");

  if (!missing.empty()) {
    std::string resolution;
    if (!db.has_point_cloud("cloud")) {
      resolution = "Run 'rux create clouds project.rux' first";
    } else {
      auto available = db.list_point_clouds();
      resolution = fmt::format("Semantic cloud '{}' not found. Available: {}",
                               semantic_cloud_name, fmt::join(available, ", "));
    }

    return ValidationResult::error(
        fmt::format("Missing required data: {}", fmt::join(missing, ", ")),
        resolution, missing);
  }

  // Size validation
  try {
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto semantic = db.point_cloud_label(semantic_cloud_name);

    if (cloud->size() != semantic->size()) {
      return ValidationResult::error(
          fmt::format("Size mismatch: cloud={}, {}={}", cloud->size(),
                      semantic_cloud_name, semantic->size()),
          "Regenerate point clouds with 'rux create clouds project.rux'", {});
    }
  } catch (const std::exception &e) {
    return ValidationResult::error(
        fmt::format("Failed to load data: {}", e.what()),
        "Check database with 'rux info project.rux'", {});
  }

  return ValidationResult::ok();
}

} // namespace rux::validation

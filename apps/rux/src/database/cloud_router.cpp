// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/cloud_router.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

namespace rux::database {

std::vector<std::string> CloudRouter::list() const {
  auto names = db_->list_point_clouds();
  std::sort(names.begin(), names.end()); // Alphabetical order (deterministic)
  return names;
}

nlohmann::json CloudRouter::get_metadata(std::string_view name) const {
  if (!db_->has_point_cloud(name)) {
    throw std::runtime_error("Point cloud not found: " + std::string(name));
  }

  nlohmann::json meta;
  meta["name"] = name;
  meta["type"] = db_->point_cloud_type(name);

  // Get point count based on type
  std::string type = meta["type"];
  size_t point_count = 0;

  if (type == "PointXYZRGB") {
    auto cloud = db_->point_cloud_xyzrgb(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "Normal") {
    auto cloud = db_->point_cloud_normal(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "Label") {
    auto cloud = db_->point_cloud_label(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "PointXYZ") {
    auto cloud = db_->point_cloud_xyz(name);
    point_count = cloud ? cloud->size() : 0;
  }

  meta["point_count"] = point_count;
  return meta;
}

std::vector<uint8_t> CloudRouter::get_cloud_binary(std::string_view name) const {
  if (!db_->has_point_cloud(name)) {
    throw std::runtime_error("Point cloud not found: " + std::string(name));
  }

  std::string type = db_->point_cloud_type(name);

  // Use temp file for PCL I/O (PCL doesn't support stream-based I/O)
  auto temp_file = std::filesystem::temp_directory_path() /
                   ("rux_cloud_" + std::string(name) + ".pcd");
  bool success = false;

  if (type == "PointXYZRGB") {
    auto cloud = db_->point_cloud_xyzrgb(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "Normal") {
    auto cloud = db_->point_cloud_normal(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "Label") {
    auto cloud = db_->point_cloud_label(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "PointXYZ") {
    auto cloud = db_->point_cloud_xyz(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else {
    throw std::runtime_error("Unknown cloud type: " + type);
  }

  if (!success) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to serialize point cloud: " +
                             std::string(name));
  }

  // Read temp file into buffer
  std::ifstream file(temp_file, std::ios::binary);
  if (!file) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to read temp file");
  }

  std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
  file.close();
  std::filesystem::remove(temp_file);

  return buffer;
}

DataPayload CloudRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of cloud names
    auto names = list();
    return nlohmann::json(names);
  }

  // Resolve index if present
  std::string item_name;

  if (components[0].is_index()) {
    // Array indexing: clouds[0]
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    // Direct item access: clouds.mycloud
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  // Check if wildcard (should have been expanded before calling this)
  if (item_name.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + item_name);
  }

  if (components.size() == 1) {
    // clouds.mycloud → return binary cloud data
    return get_cloud_binary(item_name);
  }

  // Property access: clouds.mycloud.property
  const auto &prop = components[1].value;

  if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "type") {
    return db_->point_cloud_type(item_name);
  } else if (prop == "point_count") {
    auto meta = get_metadata(item_name);
    return std::to_string(meta["point_count"].get<size_t>());
  } else {
    throw std::runtime_error("Unknown property: " + prop +
                             "\nAvailable properties: metadata, type, point_count");
  }
}

void CloudRouter::set_cloud_from_binary(std::string_view name,
                                        const std::vector<uint8_t> &data) {
  // Write data to temp file (PCL doesn't support stream-based I/O)
  auto temp_file = std::filesystem::temp_directory_path() /
                   ("rux_cloud_import_" + std::string(name) + ".pcd");

  std::ofstream file(temp_file, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to create temp file");
  }
  file.write(reinterpret_cast<const char *>(data.data()), data.size());
  file.close();

  // Try to load as PointXYZRGB (most common)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile(temp_file.string(), *cloud) == 0) {
    db_->save_point_cloud(name, *cloud);
    spdlog::info("Saved point cloud '{}' ({} points)", name, cloud->size());
    std::filesystem::remove(temp_file);
    return;
  }

  // Try other types if PointXYZRGB fails
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(temp_file.string(), *cloud_xyz) == 0) {
    db_->save_point_cloud(name, *cloud_xyz);
    spdlog::info("Saved point cloud '{}' ({} points)", name, cloud_xyz->size());
    std::filesystem::remove(temp_file);
    return;
  }

  std::filesystem::remove(temp_file);
  throw std::runtime_error("Failed to parse PCD data for cloud: " +
                           std::string(name));
}

void CloudRouter::set_cloud_from_text(std::string_view name,
                                      std::string_view data) {
  // Convert text to binary and reuse binary handler
  std::vector<uint8_t> binary(data.begin(), data.end());
  set_cloud_from_binary(name, binary);
}

void CloudRouter::set(const std::vector<PathComponent> &components,
                      const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a cloud name: clouds.<name>");
  }

  // Get item name (no index support for set)
  if (!components[0].is_item()) {
    throw std::runtime_error("Expected item name after collection");
  }

  std::string item_name = components[0].value;

  if (components.size() == 1) {
    // clouds.mycloud → set cloud data
    if (std::holds_alternative<std::vector<uint8_t>>(data)) {
      set_cloud_from_binary(item_name, std::get<std::vector<uint8_t>>(data));
    } else if (std::holds_alternative<std::string>(data)) {
      set_cloud_from_text(item_name, std::get<std::string>(data));
    } else {
      throw std::runtime_error(
          "Invalid data type for cloud. Expected binary PCD data.");
    }
  } else {
    throw std::runtime_error(
        "Cannot set individual cloud properties. Set the entire cloud.");
  }
}

void CloudRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot delete entire collection. Specify a cloud name: clouds.<name>");
  }

  if (!components[0].is_item()) {
    throw std::runtime_error("Expected item name after collection");
  }

  std::string item_name = components[0].value;

  if (!db_->has_point_cloud(item_name)) {
    throw std::runtime_error("Point cloud not found: " + item_name);
  }

  db_->delete_point_cloud(item_name);
  spdlog::info("Deleted point cloud '{}'", item_name);
}

} // namespace rux::database

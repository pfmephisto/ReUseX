// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/panorama_router.hpp"

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

namespace rux::database {

std::vector<std::string> PanoramaRouter::list() const {
  auto images = db_->list_panoramic_images();
  std::vector<std::string> names;
  names.reserve(images.size());
  for (const auto &img : images) {
    names.push_back(img.filename);
  }
  return names; // Already sorted by filename from DB query
}

nlohmann::json
PanoramaRouter::get_metadata(std::string_view filename) const {
  auto images = db_->list_panoramic_images();
  for (const auto &img : images) {
    if (img.filename == filename) {
      nlohmann::json meta;
      meta["id"] = img.id;
      meta["filename"] = img.filename;
      meta["timestamp"] = img.timestamp >= 0.0
                              ? nlohmann::json(img.timestamp)
                              : nlohmann::json(nullptr);
      meta["node_id"] = img.node_id >= 0 ? nlohmann::json(img.node_id)
                                          : nlohmann::json(nullptr);
      return meta;
    }
  }
  throw std::runtime_error("Panoramic image not found: " +
                           std::string(filename));
}

std::vector<uint8_t>
PanoramaRouter::get_image_data(std::string_view filename) const {
  cv::Mat image = db_->panoramic_image(filename);
  if (image.empty()) {
    throw std::runtime_error("Panoramic image not found: " +
                             std::string(filename));
  }

  std::vector<uint8_t> buf;
  cv::imencode(".jpg", image, buf, {cv::IMWRITE_JPEG_QUALITY, 95});
  return buf;
}

DataPayload
PanoramaRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list with metadata
    auto images = db_->list_panoramic_images();
    nlohmann::json arr = nlohmann::json::array();
    for (const auto &img : images) {
      nlohmann::json j;
      j["id"] = img.id;
      j["filename"] = img.filename;
      j["timestamp"] = img.timestamp >= 0.0 ? nlohmann::json(img.timestamp)
                                             : nlohmann::json(nullptr);
      j["node_id"] = img.node_id >= 0 ? nlohmann::json(img.node_id)
                                       : nlohmann::json(nullptr);
      arr.push_back(std::move(j));
    }
    return arr;
  }

  // Resolve item
  std::string item_name;

  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  if (item_name.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + item_name);
  }

  if (components.size() == 1) {
    // panoramas.myfile → return JPEG image data
    return get_image_data(item_name);
  }

  // Property access
  const auto &prop = components[1].value;

  if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "image") {
    return get_image_data(item_name);
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: metadata, image");
  }
}

void PanoramaRouter::set(const std::vector<PathComponent> & /*components*/,
                         const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Cannot set panoramic images via 'rux set'. "
      "Use 'rux import 360 <path>' to import panoramic images.");
}

void PanoramaRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error("Cannot delete entire collection. "
                             "Specify a filename: panoramas.<filename>");
  }

  std::string item_name;

  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item name after collection");
  }

  if (!db_->has_panoramic_image(item_name)) {
    throw std::runtime_error("Panoramic image not found: " + item_name);
  }

  db_->delete_panoramic_image(item_name);
  spdlog::info("Deleted panoramic image '{}'", item_name);
}

} // namespace rux::database

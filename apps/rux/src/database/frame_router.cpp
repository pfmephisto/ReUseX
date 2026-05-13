// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/frame_router.hpp"

#include <reusex/core/SensorIntrinsics.hpp>

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <charconv>

namespace rux::database {

namespace {

std::optional<int> parse_node_id(std::string_view s) {
  int value = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc{} || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

nlohmann::json
intrinsics_to_json(const reusex::core::SensorIntrinsics &intrinsics) {
  nlohmann::json j;
  j["fx"] = intrinsics.fx;
  j["fy"] = intrinsics.fy;
  j["cx"] = intrinsics.cx;
  j["cy"] = intrinsics.cy;
  j["width"] = intrinsics.width;
  j["height"] = intrinsics.height;
  j["local_transform"] = intrinsics.local_transform;
  return j;
}

std::vector<uint8_t> encode_png(const cv::Mat &image) {
  std::vector<uint8_t> buf;
  cv::imencode(".png", image, buf);
  return buf;
}

std::vector<uint8_t> encode_jpeg(const cv::Mat &image) {
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", image, buf, {cv::IMWRITE_JPEG_QUALITY, 95});
  return buf;
}

} // namespace

std::vector<std::string> FrameRouter::list() const {
  auto ids = db_->sensor_frame_ids();
  std::vector<std::string> names;
  names.reserve(ids.size());
  for (int id : ids) {
    names.push_back(std::to_string(id));
  }
  return names;
}

int FrameRouter::resolve_node_id(const PathComponent &component) const {
  if (component.is_index()) {
    auto resolved = resolve_index(*component.index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*component.index));
    }
    auto id = parse_node_id(*resolved);
    if (!id) {
      throw std::runtime_error("Internal error: non-numeric frame id: " +
                               *resolved);
    }
    return *id;
  }
  if (component.is_item()) {
    if (component.value.find('*') != std::string::npos) {
      throw std::runtime_error("Wildcard not expanded: " + component.value);
    }
    auto id = parse_node_id(component.value);
    if (!id) {
      throw std::runtime_error(
          "Frame must be referenced by numeric node_id, got: " +
          component.value);
    }
    if (!db_->has_sensor_frame(*id)) {
      throw std::runtime_error("Frame not found: node_id=" +
                               std::to_string(*id));
    }
    return *id;
  }
  throw std::runtime_error("Expected frame id or index after collection");
}

nlohmann::json FrameRouter::metadata_json(int nodeId) const {
  nlohmann::json j;
  j["node_id"] = nodeId;

  const double ts = db_->sensor_frame_timestamp(nodeId);
  j["timestamp"] = ts >= 0.0 ? nlohmann::json(ts) : nlohmann::json(nullptr);

  const auto intrinsics = db_->sensor_frame_intrinsics(nodeId);
  j["width"] = intrinsics.width;
  j["height"] = intrinsics.height;
  j["intrinsics"] = intrinsics_to_json(intrinsics);

  const auto pose = db_->sensor_frame_pose(nodeId);
  j["pose"] = pose;

  j["has_depth"] = !db_->sensor_frame_depth(nodeId).empty();
  j["has_confidence"] = !db_->sensor_frame_confidence(nodeId).empty();
  return j;
}

DataPayload FrameRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    auto ids = db_->sensor_frame_ids();
    nlohmann::json arr = nlohmann::json::array();
    for (int id : ids) {
      arr.push_back(id);
    }
    return arr;
  }

  const int node_id = resolve_node_id(components[0]);

  if (components.size() == 1) {
    return metadata_json(node_id);
  }

  const auto &prop = components[1].value;

  if (prop == "metadata") {
    return metadata_json(node_id);
  }
  if (prop == "color" || prop == "image") {
    cv::Mat img = db_->sensor_frame_image(node_id);
    if (img.empty()) {
      throw std::runtime_error("Frame has no color image: node_id=" +
                               std::to_string(node_id));
    }
    return encode_jpeg(img);
  }
  if (prop == "depth") {
    cv::Mat img = db_->sensor_frame_depth(node_id);
    if (img.empty()) {
      throw std::runtime_error("Frame has no depth image: node_id=" +
                               std::to_string(node_id));
    }
    return encode_png(img);
  }
  if (prop == "confidence") {
    cv::Mat img = db_->sensor_frame_confidence(node_id);
    if (img.empty()) {
      throw std::runtime_error("Frame has no confidence image: node_id=" +
                               std::to_string(node_id));
    }
    return encode_png(img);
  }
  if (prop == "pose") {
    auto pose = db_->sensor_frame_pose(node_id);
    return nlohmann::json(pose);
  }
  if (prop == "intrinsics") {
    return intrinsics_to_json(db_->sensor_frame_intrinsics(node_id));
  }
  if (prop == "timestamp") {
    const double ts = db_->sensor_frame_timestamp(node_id);
    return ts >= 0.0 ? nlohmann::json(ts) : nlohmann::json(nullptr);
  }

  throw std::runtime_error(
      "Unknown property: " + prop +
      "\nAvailable properties: metadata, color, depth, confidence, "
      "pose, intrinsics, timestamp");
}

void FrameRouter::set(const std::vector<PathComponent> & /*components*/,
                      const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Cannot set sensor frames via 'rux set'. "
      "Use 'rux import rtabmap <db>' to import sensor frames.");
}

void FrameRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error(
      "Deleting individual sensor frames is not supported.");
}

} // namespace rux::database

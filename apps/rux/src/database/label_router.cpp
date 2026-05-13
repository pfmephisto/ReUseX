// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/label_router.hpp"

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <charconv>
#include <set>

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

} // namespace

std::vector<std::string> LabelRouter::list() const {
  auto ids = db_->segmentation_image_ids();
  std::vector<std::string> names;
  names.reserve(ids.size());
  for (int id : ids) {
    names.push_back(std::to_string(id));
  }
  return names;
}

int LabelRouter::resolve_node_id(const PathComponent &component) const {
  if (component.is_index()) {
    auto resolved = resolve_index(*component.index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*component.index));
    }
    auto id = parse_node_id(*resolved);
    if (!id) {
      throw std::runtime_error("Internal error: non-numeric label id: " +
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
          "Label must be referenced by numeric node_id, got: " +
          component.value);
    }
    if (!db_->has_segmentation_image(*id)) {
      throw std::runtime_error("Label not found: node_id=" +
                               std::to_string(*id));
    }
    return *id;
  }
  throw std::runtime_error("Expected label id or index after collection");
}

std::vector<uint8_t> LabelRouter::encoded_image(int nodeId) const {
  cv::Mat labels = db_->segmentation_image(nodeId);
  if (labels.empty()) {
    throw std::runtime_error("Label image not found: node_id=" +
                             std::to_string(nodeId));
  }
  // labels is CV_32S with -1 for background. Re-apply the +1 storage offset
  // and convert to CV_16U so PNG can encode it losslessly (matching the
  // on-disk representation in ProjectDB).
  cv::Mat shifted;
  cv::add(labels, cv::Scalar(1), shifted, cv::noArray(), CV_16U);

  std::vector<uint8_t> buf;
  cv::imencode(".png", shifted, buf);
  return buf;
}

nlohmann::json LabelRouter::metadata_json(int nodeId) const {
  cv::Mat labels = db_->segmentation_image(nodeId);
  if (labels.empty()) {
    throw std::runtime_error("Label image not found: node_id=" +
                             std::to_string(nodeId));
  }

  std::set<int> classes;
  for (int y = 0; y < labels.rows; ++y) {
    const int *row = labels.ptr<int>(y);
    for (int x = 0; x < labels.cols; ++x) {
      if (row[x] >= 0) {
        classes.insert(row[x]);
      }
    }
  }

  nlohmann::json j;
  j["node_id"] = nodeId;
  j["width"] = labels.cols;
  j["height"] = labels.rows;
  j["num_classes"] = static_cast<int>(classes.size());
  j["class_ids"] = classes;
  return j;
}

DataPayload LabelRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    auto ids = db_->segmentation_image_ids();
    nlohmann::json arr = nlohmann::json::array();
    for (int id : ids) {
      arr.push_back(id);
    }
    return arr;
  }

  const int node_id = resolve_node_id(components[0]);

  if (components.size() == 1) {
    return encoded_image(node_id);
  }

  const auto &prop = components[1].value;

  if (prop == "image") {
    return encoded_image(node_id);
  }
  if (prop == "metadata") {
    return metadata_json(node_id);
  }

  throw std::runtime_error("Unknown property: " + prop +
                           "\nAvailable properties: image, metadata");
}

void LabelRouter::set(const std::vector<PathComponent> & /*components*/,
                      const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Cannot set segmentation labels via 'rux set'. "
      "Use 'rux annotate <project> --net <model>' to generate labels.");
}

void LabelRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error(
      "Deleting individual label images is not supported.");
}

} // namespace rux::database

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/describe.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "reusex/vision/IVlmClient.hpp"
#include "types.hpp"
#include "vision/instance_crop.hpp"
#include "vision/vlm/OpenAiCompatibleVlmClient.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::vision {

std::string default_describe_prompt() {
  return "You are inspecting a physical material/object in a building for "
         "reuse "
         "and material assessment. Look at the object in the image and respond "
         "with ONLY a JSON object (no prose, no markdown fences) of the form "
         "{\"description\": \"<a short free-text description of the material "
         "and its condition>\", \"attributes\": {\"<key>\": \"<value>\", "
         "...}}. "
         "Put a free-text summary in \"description\" and any concrete "
         "properties you can read off the image (for example material, colour, "
         "condition, dimensions) as key/value pairs under \"attributes\". Use "
         "whatever keys fit what you can actually see. Do not invent details "
         "you cannot see; omit a key rather than guessing.";
}

namespace {

// Gather world-space points per instance id (label >= 1), subsampling to at
// most `cap` points each with a deterministic stride so a huge instance does
// not dominate the reprojection cost. Requires the label cloud and the XYZRGB
// cloud to be index-aligned (STANDARDS §3.2).
std::map<int, std::vector<Eigen::Vector3f>>
gather_instance_points(const Cloud &xyz, const CloudL &labels,
                       std::size_t cap) {
  std::map<int, std::vector<Eigen::Vector3f>> out;
  const std::size_t n = std::min(xyz.points.size(), labels.points.size());
  for (std::size_t i = 0; i < n; ++i) {
    const uint32_t lab = labels.points[i].label;
    if (lab == 0) // 0 == unlabeled (STANDARDS §3.1)
      continue;
    const auto &p = xyz.points[i];
    out[static_cast<int>(lab)].emplace_back(p.x, p.y, p.z);
  }
  // Deterministic subsample per instance.
  for (auto &[id, pts] : out) {
    (void)id;
    if (pts.size() <= cap)
      continue;
    const std::size_t stride = pts.size() / cap;
    std::vector<Eigen::Vector3f> kept;
    kept.reserve(cap);
    for (std::size_t i = 0; i < pts.size() && kept.size() < cap; i += stride)
      kept.push_back(pts[i]);
    pts.swap(kept);
  }
  return out;
}

} // namespace

int describe_with_client(const std::filesystem::path &dbPath,
                         const DescribeConfig &config, IVlmClient &client) {
  ProjectDB db(dbPath);

  if (!db.has_point_cloud(config.instances_cloud))
    throw std::runtime_error("describe (attributes): instance-label cloud '" +
                             config.instances_cloud +
                             "' not found — run `rux create instances` first");
  if (!db.has_point_cloud(config.point_cloud))
    throw std::runtime_error("describe (attributes): point cloud '" +
                             config.point_cloud +
                             "' not found — run `rux create clouds` first");

  auto labels = db.point_cloud_label(config.instances_cloud);
  auto xyz = db.point_cloud_xyzrgb(config.point_cloud);
  if (!labels || !xyz)
    throw std::runtime_error(
        "describe (attributes): failed to load instance/point clouds");
  if (labels->points.size() != xyz->points.size())
    throw std::runtime_error("describe (attributes): instance cloud (" +
                             std::to_string(labels->points.size()) +
                             " pts) and point cloud (" +
                             std::to_string(xyz->points.size()) +
                             " pts) are not index-aligned (STANDARDS §3.2)");

  // The materials to describe: every material passport in the project.
  const auto material_guids = db.list_passport_guids();
  if (material_guids.empty()) {
    reusex::warn("describe (attributes): no material passports in project — "
                 "nothing to describe (run `rux create materials` first)");
    return 0;
  }

  // material_guid -> the instance ids linked to it (a material may back many).
  std::map<std::string, std::vector<int>> material_instances;
  for (const auto &[instance_id, guid] :
       db.instance_materials(config.instances_cloud))
    material_instances[guid].push_back(instance_id);

  auto instance_points =
      gather_instance_points(*xyz, *labels, config.max_points_per_frame);

  const auto frame_ids = db.sensor_frame_ids();
  if (frame_ids.empty()) {
    reusex::warn("describe (attributes): no sensor frames in project — cannot "
                 "crop any material view; {} materials left undescribed",
                 material_guids.size());
    return 0;
  }

  // Pre-load frame poses/intrinsics once (skip poseless frames — projecting
  // through the identity fallback would crop the wrong pixels, #336).
  struct FrameView {
    int id;
    std::array<double, 16> pose;
    core::SensorIntrinsics intr;
  };
  std::vector<FrameView> views;
  views.reserve(frame_ids.size());
  for (int id : frame_ids) {
    if (!db.has_sensor_frame_pose(id))
      continue;
    views.push_back(
        {id, db.sensor_frame_pose(id), db.sensor_frame_intrinsics(id)});
  }
  if (views.empty()) {
    reusex::warn("describe (attributes): none of the {} sensor frames carry a "
                 "usable pose; {} materials left undescribed",
                 frame_ids.size(), material_guids.size());
    return 0;
  }

  const std::string prompt =
      config.prompt.empty() ? default_describe_prompt() : config.prompt;
  const std::string provider = config.api_url + "|" + config.model;

  int stored = 0;
  int no_instance = 0;
  int no_view = 0;
  int model_empty = 0;
  int call_failed = 0;

  for (const auto &material_guid : material_guids) {
    if (config.skip_existing && db.material_annotation(material_guid))
      continue;

    // Resolve the material to its linked instance(s), then to their 3D points.
    auto lit = material_instances.find(material_guid);
    if (lit == material_instances.end() || lit->second.empty()) {
      ++no_instance;
      reusex::warn("Material {}: no linked instance in cloud '{}', skipping",
                   material_guid, config.instances_cloud);
      continue;
    }

    std::vector<Eigen::Vector3f> pts;
    for (int instance_id : lit->second) {
      auto pit = instance_points.find(instance_id);
      if (pit != instance_points.end())
        pts.insert(pts.end(), pit->second.begin(), pit->second.end());
    }
    if (pts.empty()) {
      ++no_instance;
      reusex::warn("Material {}: its linked instance(s) have no points in "
                   "cloud '{}', skipping",
                   material_guid, config.instances_cloud);
      continue;
    }

    // Find the frame with the most in-bounds projected points (best view)
    // across all of the material's instances. Selection uses each frame's
    // native intrinsic resolution; the winner is re-projected against the
    // actual stored image size below in case they differ.
    const FrameView *best_view = nullptr;
    ProjectedBox best_box;
    for (const auto &view : views) {
      auto box = project_points_to_box(pts, view.pose, view.intr,
                                       view.intr.width, view.intr.height);
      if (box && box->in_bounds > best_box.in_bounds) {
        best_box = *box;
        best_view = &view;
      }
    }

    if (best_view == nullptr || best_box.in_bounds < config.min_view_points) {
      ++no_view;
      reusex::debug("Material {}: no frame with >= {} in-bounds points "
                    "(best {}), skipping",
                    material_guid, config.min_view_points, best_box.in_bounds);
      continue;
    }

    cv::Mat image = db.sensor_frame_image(best_view->id); // CV_8UC3 BGR
    if (image.empty()) {
      ++no_view;
      reusex::warn("Material {}: best frame {} has no color image, skipping",
                   material_guid, best_view->id);
      continue;
    }

    // Re-project against the actual image size (intrinsics are scaled inside
    // project_points_to_box) so the crop box lines up with the stored image.
    auto box = project_points_to_box(pts, best_view->pose, best_view->intr,
                                     image.cols, image.rows);
    if (!box) {
      ++no_view;
      continue;
    }

    const int pad = std::max(0, config.crop_padding);
    const int x0 = std::max(0, box->x_min - pad);
    const int y0 = std::max(0, box->y_min - pad);
    const int x1 = std::min(image.cols - 1, box->x_max + pad);
    const int y1 = std::min(image.rows - 1, box->y_max + pad);
    if (x1 <= x0 || y1 <= y0) {
      ++no_view;
      continue;
    }
    cv::Mat crop = image(cv::Rect(x0, y0, x1 - x0, y1 - y0)).clone();

    VlmResult result;
    try {
      result = client.describe(crop, prompt);
    } catch (const std::exception &e) {
      ++call_failed;
      reusex::warn(
          "Material {}: VLM call failed ({}); leaving annotation unset",
          material_guid, e.what());
      continue;
    }

    if (!result.ok) {
      ++model_empty;
      reusex::warn("Material {}: VLM returned no usable description or "
                   "attributes; leaving annotation unset (raw kept for "
                   "debugging)",
                   material_guid);
      // Do not fabricate: skip the save entirely (STANDARDS §5).
      continue;
    }

    ProjectDB::MaterialAnnotation annotation;
    annotation.description = result.description;
    annotation.attributes = result.attributes;
    annotation.provider_model = provider;
    annotation.raw_json = result.raw_json;
    db.save_material_annotation(material_guid, annotation);
    ++stored;
  }

  if (no_instance > 0 || no_view > 0 || model_empty > 0 || call_failed > 0)
    reusex::warn("describe (attributes): stored {} / {} materials "
                 "({} no linked instance, {} no usable view, {} unusable model "
                 "output, {} call failures)",
                 stored, material_guids.size(), no_instance, no_view,
                 model_empty, call_failed);
  else
    reusex::info("describe (attributes): stored annotations for {} / {} "
                 "materials",
                 stored, material_guids.size());

  return stored;
}

int describe(const std::filesystem::path &dbPath,
             const DescribeConfig &config) {
  try {
    OpenAiCompatibleVlmClient client(config.api_url, config.model,
                                     config.api_key);
    describe_with_client(dbPath, config, client);
    return 0;
  } catch (const std::exception &e) {
    reusex::error("describe (attributes) failed: {}", e.what());
    return 1;
  }
}

} // namespace reusex::vision

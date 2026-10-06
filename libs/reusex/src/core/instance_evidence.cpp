// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/instance_evidence.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace reusex::core {

namespace {

using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

bool finite(const std::array<double, 16> &pose) {
  return std::all_of(pose.begin(), pose.end(),
                     [](double v) { return std::isfinite(v); });
}

} // namespace

std::map<std::uint32_t, InstanceCentroid>
instance_centroids(const ProjectDB &db, std::string_view label_cloud,
                   std::string_view positions_cloud) {
  if (!db.has_point_cloud(label_cloud))
    throw std::invalid_argument("instance_centroids: no point cloud '" +
                                std::string(label_cloud) + "'");
  if (!db.has_point_cloud(positions_cloud))
    throw std::invalid_argument("instance_centroids: no positions cloud '" +
                                std::string(positions_cloud) + "'");

  const auto positions = db.point_cloud_xyzrgb(positions_cloud);
  const auto labels = db.point_cloud_label(label_cloud);
  if (!positions || !labels)
    throw std::runtime_error("instance_centroids: could not load '" +
                             std::string(positions_cloud) + "' or '" +
                             std::string(label_cloud) + "'");
  if (positions->size() != labels->size())
    throw std::runtime_error(
        "instance-label cloud '" + std::string(label_cloud) + "' has " +
        std::to_string(labels->size()) + " points but base cloud '" +
        std::string(positions_cloud) + "' has " +
        std::to_string(positions->size()) + " — they are not index-aligned");

  struct Sum {
    double x = 0, y = 0, z = 0;
    std::size_t n = 0;
  };
  std::map<std::uint32_t, Sum> sums;
  for (std::size_t i = 0; i < labels->size(); ++i) {
    const auto label = labels->points[i].label;
    if (label == 0) // unlabeled (STANDARDS §3)
      continue;
    const auto &p = positions->points[i];
    // One NaN must not poison the average.
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    auto &s = sums[label];
    s.x += p.x;
    s.y += p.y;
    s.z += p.z;
    ++s.n;
  }

  std::map<std::uint32_t, InstanceCentroid> out;
  for (const auto &[label, s] : sums) {
    const double n = static_cast<double>(s.n);
    out.emplace(label, InstanceCentroid{
                           Eigen::Vector3d(s.x / n, s.y / n, s.z / n), s.n});
  }
  return out;
}

std::map<std::string, PartPhotos>
survey_part_photos(const ProjectDB &db, const VisibilityQuery &query,
                   std::string_view positions_cloud) {
  // Instance-backed parts, grouped by the cloud their instance lives in.
  std::map<std::string, std::vector<std::pair<std::string, std::uint32_t>>>
      by_cloud;
  for (const auto &part : db.survey_parts()) {
    if (!part.cloud_name || !part.instance_id || *part.instance_id == 0)
      continue;
    by_cloud[*part.cloud_name].emplace_back(part.code, *part.instance_id);
  }

  std::map<std::string, PartPhotos> out;
  if (by_cloud.empty())
    return out;

  // Every centroid of every cloud, in one list, so the frames are read once.
  std::vector<Eigen::Vector3d> points;
  std::vector<std::string> owners; // part code per point
  for (const auto &[cloud, parts] : by_cloud) {
    std::map<std::uint32_t, InstanceCentroid> centroids;
    try {
      centroids = instance_centroids(db, cloud, positions_cloud);
    } catch (const std::exception &e) {
      reusex::warn("survey_part_photos: skipping {} part(s) on cloud '{}': {}",
                   parts.size(), cloud, e.what());
      continue;
    }
    for (const auto &[code, id] : parts) {
      const auto it = centroids.find(id);
      if (it == centroids.end())
        continue; // the instance has no points (orphaned part)
      points.push_back(it->second.centroid);
      owners.push_back(code);
    }
  }

  const auto ranked = visible_frames_batch(db, points, query);
  for (std::size_t i = 0; i < ranked.size(); ++i) {
    PartPhotos photos;
    photos.count = ranked[i].size();
    if (!ranked[i].empty())
      photos.best_frame_id = ranked[i].front().frame_id;
    out.emplace(owners[i], photos);
  }
  return out;
}

Eigen::Vector2d equirect_uv(const Eigen::Vector3d &bearing) {
  const double length = bearing.norm();
  if (!(length > 0.0))
    return {0.5, 0.5};
  const Eigen::Vector3d d = bearing / length;
  const double theta = std::atan2(d.x(), d.z());
  const double phi = std::asin(std::clamp(-d.y(), -1.0, 1.0));
  return {(theta + std::numbers::pi) / (2.0 * std::numbers::pi),
          (std::numbers::pi / 2.0 - phi) / std::numbers::pi};
}

std::vector<PanoramaSighting>
panoramas_for_point(const ProjectDB &db, const Eigen::Vector3d &world_point,
                    const PanoramaQuery &query) {
  const auto panoramas = db.list_panoramic_images();

  // The levelled, heading-free basis (frontend `LEVELLED_BASIS`): columns
  // are the panorama's +x (right), +y (down), +z (forward) in world.
  Eigen::Matrix3d levelled;
  levelled.col(0) = Eigen::Vector3d(0, -1, 0);
  levelled.col(1) = Eigen::Vector3d(0, 0, -1);
  levelled.col(2) = Eigen::Vector3d(1, 0, 0);

  std::vector<PanoramaSighting> out;
  std::size_t placeable = 0;
  for (const auto &pano : panoramas) {
    Eigen::Vector3d centre;
    Eigen::Matrix3d basis; // panorama -> world
    PanoramaHeading heading = PanoramaHeading::resected;
    if (pano.has_pose && finite(pano.pose)) {
      const Matrix4dRM pose = Eigen::Map<const Matrix4dRM>(pano.pose.data());
      centre = pose.block<3, 1>(0, 3);
      basis = pose.block<3, 3>(0, 0);
    } else if (pano.node_id >= 0 && db.has_sensor_frame_pose(pano.node_id)) {
      const auto frame_pose = db.sensor_frame_pose(pano.node_id);
      if (!finite(frame_pose))
        continue;
      centre = Eigen::Vector3d(frame_pose[3], frame_pose[7], frame_pose[11]);
      basis = levelled;
      heading = PanoramaHeading::levelled;
    } else {
      continue;
    }
    ++placeable;

    const Eigen::Vector3d offset = world_point - centre;
    const double distance = offset.norm();
    if (query.max_distance > 0.0 && distance > query.max_distance)
      continue;
    const Eigen::Vector2d uv = equirect_uv(basis.transpose() * offset);
    out.push_back(PanoramaSighting{pano.id, pano.node_id, distance, uv.x(),
                                   uv.y(), heading});
  }

  std::sort(out.begin(), out.end(),
            [](const PanoramaSighting &a, const PanoramaSighting &b) {
              if (a.distance != b.distance)
                return a.distance < b.distance;
              return a.panorama_id < b.panorama_id;
            });

  if (!panoramas.empty() && placeable == 0) {
    reusex::warn("panoramas_for_point: none of {} panoramas can be placed (no "
                 "aligned pose and no posed matched frame) — 0 panoramas near "
                 "({:.3f}, {:.3f}, {:.3f})",
                 panoramas.size(), world_point.x(), world_point.y(),
                 world_point.z());
  } else {
    reusex::debug("panoramas_for_point: {}/{} placeable panoramas within {} m "
                  "of ({:.3f}, {:.3f}, {:.3f})",
                  out.size(), placeable, query.max_distance, world_point.x(),
                  world_point.y(), world_point.z());
  }
  return out;
}

} // namespace reusex::core

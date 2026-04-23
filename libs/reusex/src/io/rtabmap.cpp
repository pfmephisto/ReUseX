// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rtabmap.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>

#include <map>

namespace reusex::io {

void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path) {
  core::info("Importing RTABMap database: {}", rtabmap_db_path);
  core::stopwatch timer;

  // ── Initialise RTABMap and load the optimised graph ──────────────
  rtabmap::ParametersMap params;
  rtabmap::Rtabmap rtabmap;
  rtabmap.init(params, rtabmap_db_path.c_str());
  rtabmap.setWorkingDirectory("./");
  core::debug("RTABMap initialized in {:.3f}s", timer);

  timer.reset();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  std::map<int, rtabmap::Signature> nodes;
  rtabmap.getGraph(poses, links,
                   /*optimized=*/true, /*global=*/true, &nodes,
                   /*withImages=*/true, /*withScan=*/true,
                   /*withUserData=*/true, /*withGrid=*/true);
  core::debug("Graph loaded in {:.3f}s ({} nodes)", timer, poses.size());

  // ── Import each node ─────────────────────────────────────────────
  auto observer =
      core::ProgressObserver(core::Stage::importing_data, poses.size());

  int logId = db.log_pipeline_start(
      "import", fmt::format(R"({{"source":"{}"}})", rtabmap_db_path.string()));

  try {
    for (const auto &[id, pose] : poses) {
      auto nodeIt = nodes.find(id);
      if (nodeIt == nodes.end()) {
        core::warn("Node {} present in poses but missing from signatures, "
                   "skipping",
                   id);
        ++observer;
        continue;
      }

      rtabmap::SensorData data = nodeIt->second.sensorData();
      data.uncompressData();

      // ── Extract raw sensor data ──────────────────────────────────
      cv::Mat color = data.imageRaw();
      cv::Mat depth = data.depthOrRightRaw();
      cv::Mat confidence = data.depthConfidenceRaw();

      if (color.empty()) {
        core::debug("Node {} has no color image, skipping", id);
        ++observer;
        continue;
      }

      // Store images in original RTABMap orientation (no rotation).
      // Any rotation for display should be handled at visualization time.

      // ── Camera intrinsics ────────────────────────────────────────
      core::SensorIntrinsics intrinsics;
      if (!data.cameraModels().empty()) {
        const auto &cm = data.cameraModels().front();
        // Store intrinsics as-is from RTABMap
        intrinsics.fx = cm.fx();
        intrinsics.fy = cm.fy();
        intrinsics.cx = cm.cx();
        intrinsics.cy = cm.cy();
        intrinsics.width = cm.imageWidth();
        intrinsics.height = cm.imageHeight();

        // Convert local transform to row-major array
        const auto &lt = cm.localTransform();
        if (!lt.isNull()) {
          const float *d = lt.data();
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
              intrinsics.local_transform[r * 4 + c] =
                  static_cast<double>(d[r * 4 + c]);
          intrinsics.local_transform[12] = 0;
          intrinsics.local_transform[13] = 0;
          intrinsics.local_transform[14] = 0;
          intrinsics.local_transform[15] = 1;
        }
      }

      // ── World pose as row-major 4x4 ─────────────────────────────
      std::array<double, 16> worldPose;
      if (!pose.isNull()) {
        const float *d = pose.data();
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 4; ++c)
            worldPose[r * 4 + c] = static_cast<double>(d[r * 4 + c]);
        worldPose[12] = 0;
        worldPose[13] = 0;
        worldPose[14] = 0;
        worldPose[15] = 1;
      } else {
        worldPose = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
      }

      // ── Store in ProjectDB ───────────────────────────────────────
      db.save_sensor_frame(id, color, depth, confidence, worldPose, intrinsics);

      core::trace("Imported node {} ({}x{}, depth {}x{})", id, color.cols,
                  color.rows, depth.cols, depth.rows);
      ++observer;
    }

    db.log_pipeline_end(logId, true);
  } catch (...) {
    db.log_pipeline_end(logId, false, "import_rtabmap failed");
    rtabmap.close();
    throw;
  }

  rtabmap.close();
  core::info("Import complete: {} sensor frames stored",
             db.sensor_frame_ids().size());
}

} // namespace reusex::io

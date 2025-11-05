// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/annotate.hpp"

#include "ReUseX/utils/fmt_formatter.hpp"
#include "ReUseX/vision/annotate.hpp"
#include "spdmon/spdmon.hpp"

#include <sqlite3.h>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <filesystem>

namespace fs = std::filesystem;
using namespace rtabmap;

void setup_subcommand_annotate(CLI::App &app) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandAnnotateOptions>();
  auto *sub = app.add_subcommand(
      "annotate", "Annotate an RTAB-Map database and "
                  "export a point cloud with labels and normals, "
                  "and the trajectory.");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-n, --net", opt->net_path, "Path to the YOLOv8 ONNX model")
      //->check(CLI::ExistingFile)
      ->default_val(opt->net_path);

  sub->add_flag("-c, --cuda", opt->isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(opt->isCuda);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_annotate");
    return run_subcommand_annotate(*opt);
  });
}

int run_subcommand_annotate(SubcommandAnnotateOptions const &opt) {
  return ReUseX::vision::annotateRTABMap(opt.database_path_in, opt.net_path,
                                         opt.isCuda);
}

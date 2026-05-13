// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "filter/downsample.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/downsample.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace {

// Parse a --with entry "NAME" or "NAME:OUT_NAME" into (input, output).
// Empty OUT_NAME means overwrite the input cloud.
std::pair<std::string, std::string>
parse_with_entry(const std::string &entry) {
  auto colon = entry.find(':');
  if (colon == std::string::npos)
    return {entry, entry};
  std::string in = entry.substr(0, colon);
  std::string out = entry.substr(colon + 1);
  if (in.empty())
    throw std::invalid_argument("--with entry has empty input name: '" + entry +
                                "'");
  if (out.empty())
    out = in;
  return {std::move(in), std::move(out)};
}

} // namespace

void setup_subcommand_filter_downsample(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandFilterDownsampleOptions>();
  auto *sub =
      app.add_subcommand("downsample", "Voxel-grid downsample a project cloud");

  sub->footer(R"(
DESCRIPTION:
  Downsamples an XYZRGB point cloud already stored in the project database
  using a voxel grid. Each occupied voxel contributes one centroid point
  in the output. Unlike pcl::VoxelGrid (which uses int32 voxel indices and
  overflows around 2 billion total voxels), this command uses int64
  voxel coordinates and a hash-based partition, so it scales to multi-
  billion-point inputs.

  Use --with to apply the same voxel partition to a parallel cloud, e.g.
  a sibling "normals" cloud with the same row count, so the downsampled
  outputs stay row-aligned with the primary cloud.

EXAMPLES:
  rux filter downsample -r 0.05
      Overwrite 'cloud' with a 5 cm voxel-downsampled version.

  rux filter downsample -r 0.05 --with normals
      Also overwrite 'normals' using the same partition.

  rux filter downsample -i cloud -o cloud_5cm -r 0.05 --with normals:normals_5cm
      Write to new names 'cloud_5cm' and 'normals_5cm', leaving the
      originals untouched.

  rux filter downsample -r 0.02 --with normals --with labels
      (--with is repeatable.)

NOTES:
  - The input cloud must have type PointXYZRGB.
  - Each parallel --with cloud must have exactly the same row count as
    the input (which is the case for clouds saved alongside it by the
    importer or reconstructor).
  - Supported parallel cloud types: PointXYZRGB, Normal.
)");

  sub->add_option("-i,--input", opt->input,
                  "Source cloud name in the project DB")
      ->default_val(opt->input);

  sub->add_option(
      "-o,--output", opt->output,
      "Output cloud name (defaults to the input name — overwrites it)");

  sub->add_option("-r,--resolution", opt->resolution,
                  "Voxel leaf size in meters")
      ->required()
      ->check(CLI::PositiveNumber);

  sub->add_option(
         "--with", opt->with,
         "Parallel cloud to downsample with the same voxel partition. "
         "Format: NAME or NAME:OUT_NAME. Repeatable.")
      ->take_all();

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_filter_downsample");
    return run_subcommand_filter_downsample(*opt, *global_opt);
  });
}

int run_subcommand_filter_downsample(
    SubcommandFilterDownsampleOptions const &opt,
    const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  const std::string output_name = opt.output.empty() ? opt.input : opt.output;

  try {
    reusex::ProjectDB db(project_path);

    if (!db.has_point_cloud(opt.input)) {
      spdlog::error("Input cloud '{}' not found in {}", opt.input,
                    project_path.string());
      spdlog::info("Resolution: run 'rux info' to list available clouds");
      return RuxError::INVALID_ARGUMENT;
    }

    const std::string input_type = db.point_cloud_type(opt.input);
    if (input_type != "PointXYZRGB") {
      spdlog::error("Input cloud '{}' has type {}, expected PointXYZRGB",
                    opt.input, input_type);
      return RuxError::INVALID_ARGUMENT;
    }

    // Parse and validate every --with entry up front so we fail fast.
    struct WithEntry {
      std::string input;
      std::string output;
      std::string type;
    };
    std::vector<WithEntry> with_pairs;
    with_pairs.reserve(opt.with.size());
    for (const auto &entry : opt.with) {
      auto pair = parse_with_entry(entry);
      if (!db.has_point_cloud(pair.first)) {
        spdlog::error("Parallel cloud '{}' (from --with '{}') not found",
                      pair.first, entry);
        return RuxError::INVALID_ARGUMENT;
      }
      std::string t = db.point_cloud_type(pair.first);
      if (t != "Normal" && t != "PointXYZRGB") {
        spdlog::error("Parallel cloud '{}' has unsupported type {}; "
                      "--with currently supports PointXYZRGB and Normal",
                      pair.first, t);
        return RuxError::INVALID_ARGUMENT;
      }
      with_pairs.push_back({std::move(pair.first), std::move(pair.second),
                            std::move(t)});
    }

    spdlog::info("Loading input cloud '{}' from {}", opt.input,
                 project_path.string());
    auto cloud = db.point_cloud_xyzrgb(opt.input);
    if (!cloud || cloud->empty()) {
      spdlog::error("Input cloud '{}' is empty", opt.input);
      return RuxError::INVALID_ARGUMENT;
    }
    spdlog::info("Input: {} points", cloud->size());

    const std::string params_json = fmt::format(
        R"({{"input":"{}","resolution":{},"source_points":{}}})", opt.input,
        opt.resolution, cloud->size());

    int logId = db.log_pipeline_start("filter_downsample", params_json);

    try {
      spdlog::info("Building voxel assignment (leaf={} m)", opt.resolution);
      auto assignment =
          reusex::geometry::voxel_assignment(*cloud, opt.resolution);

      spdlog::info("Reducing primary cloud ({} buckets)",
                   assignment.bucket_count);
      auto down = reusex::geometry::downsample(*cloud, assignment);
      spdlog::info("Saving '{}' ({} points)", output_name, down->size());
      db.save_point_cloud(output_name, *down, "filter_downsample", params_json);

      // Free the primary input cloud as early as possible — for large
      // inputs holding both in memory while filtering 'with' clouds risks
      // OOM. The assignment is the only thing we still need.
      cloud.reset();
      down.reset();

      for (const auto &w : with_pairs) {
        spdlog::info("Loading parallel cloud '{}' (type {})", w.input, w.type);
        if (w.type == "Normal") {
          auto normals = db.point_cloud_normal(w.input);
          if (!normals)
            throw std::runtime_error("Parallel cloud '" + w.input +
                                     "' could not be loaded");
          if (normals->size() != assignment.point_to_bucket.size()) {
            throw std::runtime_error(fmt::format(
                "Parallel cloud '{}' has {} rows, expected {} (input row "
                "count); --with requires row-aligned siblings",
                w.input, normals->size(),
                assignment.point_to_bucket.size()));
          }
          auto ds = reusex::geometry::downsample(*normals, assignment);
          spdlog::info("Saving '{}' ({} entries)", w.output, ds->size());
          db.save_point_cloud(w.output, *ds, "filter_downsample", params_json);
        } else { // "PointXYZRGB" — already validated
          auto par = db.point_cloud_xyzrgb(w.input);
          if (!par)
            throw std::runtime_error("Parallel cloud '" + w.input +
                                     "' could not be loaded");
          if (par->size() != assignment.point_to_bucket.size()) {
            throw std::runtime_error(fmt::format(
                "Parallel cloud '{}' has {} rows, expected {} (input row "
                "count); --with requires row-aligned siblings",
                w.input, par->size(), assignment.point_to_bucket.size()));
          }
          auto ds = reusex::geometry::downsample(*par, assignment);
          spdlog::info("Saving '{}' ({} entries)", w.output, ds->size());
          db.save_point_cloud(w.output, *ds, "filter_downsample", params_json);
        }
      }

      db.log_pipeline_end(logId, true);
    } catch (...) {
      db.log_pipeline_end(logId, false, "filter_downsample failed");
      throw;
    }

    spdlog::info("Downsample complete");
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Downsample failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

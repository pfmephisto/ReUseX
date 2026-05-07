// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/ply.hpp"
#include "filter_utils.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/ply.hpp>

#include <pcl/common/io.h>
#include <spdlog/spdlog.h>

void setup_subcommand_export_ply(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandExportPLYOptions>();
  auto *sub = parent.add_subcommand("ply", "Export point cloud to a PLY file");

  sub->footer(R"(
DESCRIPTION:
  Exports a named XYZRGB point cloud from the project database to a binary
  PLY file. If a matching normals cloud exists ({name}_normals), normals are
  merged into the output as additional fields. An optional filter expression
  subsets the exported points using label clouds already in the project.

EXAMPLES:
  rux export ply                                  # Export 'cloud' to cloud.ply
  rux export ply -o scan.ply                      # Custom output path
  rux export ply --name scan_0 -o scan_0.ply      # Different source cloud
  rux export ply -f 'room_labels in [1,2]'        # Only rooms 1 and 2
  rux export ply -f 'plane_labels == 3' -o wall.ply

FILTER SYNTAX:
  <label_cloud> <op> <value(s)>
  -f 'room_labels in [1,2,5]'      # Labels 1, 2 or 5
  -f 'planes == 3'                 # Exact match
  -f 'planes >= 10 && planes <= 20' # Range

NOTES:
  - Output is always binary PLY (little-endian)
  - Normals cloud must have the same point count as the source cloud
  - Filter references label clouds by name (e.g. 'room_labels', 'planes')
)");

  sub->add_option("--name", opt->cloud_name,
                  "Name of the point cloud in ProjectDB")
      ->default_val(opt->cloud_name);

  sub->add_option("-o,--output", opt->output_path,
                  "Output PLY file path (default: {name}.ply)");

  sub->add_option("-f,--filter", opt->filter_expr,
                  "Filter expression to limit exported points.\n"
                  "Syntax: <cloud_name> <op> <value(s)>\n"
                  "Examples:\n"
                  "  -f 'room_labels in [1,2]'\n"
                  "  -f 'plane_labels == 3'")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_export_ply");
    return run_subcommand_export_ply(*opt, *global_opt);
  });
}

int run_subcommand_export_ply(SubcommandExportPLYOptions const &opt, const RuxOptions &global_opt) {
  try {
    spdlog::info("Exporting PLY from project: {}", global_opt.project_db.string());

    reusex::ProjectDB db(global_opt.project_db);

    if (!db.has_point_cloud(opt.cloud_name)) {
      spdlog::error("Point cloud '{}' not found in project", opt.cloud_name);
      auto names = db.list_point_clouds();
      if (!names.empty()) {
        std::string list;
        for (const auto &n : names) { if (!list.empty()) list += ", "; list += n; }
        spdlog::info("Available clouds: {}", list);
      }
      return RuxError::INVALID_ARGUMENT;
    }

    auto cloud   = db.point_cloud_xyzrgb(opt.cloud_name);
    std::string normals_name = opt.cloud_name + "_normals";
    auto normals = db.has_point_cloud(normals_name)
                 ? db.point_cloud_normal(normals_name) : nullptr;

    // Apply filter if provided
    reusex::IndicesPtr indices;
    if (!opt.filter_expr.empty()) {
      try {
        indices = rux::filters::evaluate_filter(opt.filter_expr, db, cloud->size());
      } catch (const std::exception &e) {
        spdlog::error("Filter evaluation failed: {}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    // Build export clouds (filtered or full)
    Cloud export_cloud;
    CloudN export_normals;
    const CloudN *normals_ptr = nullptr;

    if (indices) {
      pcl::copyPointCloud(*cloud, *indices, export_cloud);
      if (normals) {
        pcl::copyPointCloud(*normals, *indices, export_normals);
        normals_ptr = &export_normals;
      }
    } else {
      export_cloud = *cloud;
      if (normals) normals_ptr = normals.get();
    }

    // Resolve output path
    fs::path out = opt.output_path.empty()
                 ? fs::path(opt.cloud_name + ".ply")
                 : opt.output_path;

    reusex::io::export_ply(out, export_cloud, normals_ptr);

    spdlog::info("Exported {} points to {}", export_cloud.size(), out.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("PLY export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

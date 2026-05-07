// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/e57.hpp"
#include "filter_utils.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/e57.hpp>

#include <pcl/common/io.h>
#include <spdlog/spdlog.h>

void setup_subcommand_export_e57(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandExportE57Options>();
  auto *sub = parent.add_subcommand("e57", "Export point cloud to an E57 file");

  sub->footer(R"(
DESCRIPTION:
  Exports a named XYZRGB point cloud from the project database to an E57
  file. If a matching normals cloud exists ({name}_normals), normals are
  included using the E57_EXT_surface_normals extension. An optional filter
  expression subsets the exported points using label clouds already in the
  project.

EXAMPLES:
  rux export e57                                   # Export 'cloud' to cloud.e57
  rux export e57 -o scan.e57                       # Custom output path
  rux export e57 --name scan_0 -o scan_0.e57       # Different source cloud
  rux export e57 -f 'room_labels in [1,2]'         # Only rooms 1 and 2
  rux export e57 -f 'plane_labels == 3' -o wall.e57

FILTER SYNTAX:
  <label_cloud> <op> <value(s)>
  -f 'room_labels in [1,2,5]'      # Labels 1, 2 or 5
  -f 'planes == 3'                 # Exact match
  -f 'planes >= 10 && planes <= 20' # Range

NOTES:
  - Scan name in the E57 file matches the cloud name in ProjectDB
  - Normals cloud must have the same point count as the source cloud
  - Filter references label clouds by name (e.g. 'room_labels', 'planes')
)");

  sub->add_option("--name", opt->cloud_name,
                  "Name of the point cloud in ProjectDB")
      ->default_val(opt->cloud_name);

  sub->add_option("-o,--output", opt->output_path,
                  "Output E57 file path (default: {name}.e57)");

  sub->add_option("-f,--filter", opt->filter_expr,
                  "Filter expression to limit exported points.\n"
                  "Syntax: <cloud_name> <op> <value(s)>\n"
                  "Examples:\n"
                  "  -f 'room_labels in [1,2]'\n"
                  "  -f 'plane_labels == 3'")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_export_e57");
    return run_subcommand_export_e57(*opt, *global_opt);
  });
}

int run_subcommand_export_e57(SubcommandExportE57Options const &opt, const RuxOptions &global_opt) {
  try {
    spdlog::info("Exporting E57 from project: {}", global_opt.project_db.string());

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
    CloudPtr export_cloud = std::make_shared<Cloud>();
    CloudNPtr export_normals;

    if (indices) {
      pcl::copyPointCloud(*cloud, *indices, *export_cloud);
      if (normals) {
        export_normals = std::make_shared<CloudN>();
        pcl::copyPointCloud(*normals, *indices, *export_normals);
      }
    } else {
      export_cloud = cloud;
      export_normals = normals;
    }

    // Resolve output path
    fs::path out = opt.output_path.empty()
                 ? fs::path(opt.cloud_name + ".e57")
                 : opt.output_path;

    reusex::io::export_e57(out, {{opt.cloud_name, export_cloud, export_normals}});

    spdlog::info("Exported {} points to {}", export_cloud->size(), out.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("E57 export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

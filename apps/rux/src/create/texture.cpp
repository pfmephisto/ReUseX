// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/texture.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/unweld.hpp>
#include <reusex/reconstruction/texture_mesh.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <fmt/format.h>

#include <Eigen/Dense>

void setup_subcommand_create_texture(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandTextureOptions>();
  auto *sub = app.add_subcommand(
      "texture", "Apply textures to mesh from point cloud colors");

  sub->footer(R"(
DESCRIPTION:
  Projects point cloud RGB colors onto 3D mesh surfaces using spatial
  proximity and normal filtering. Generates UV-mapped textured mesh with
  adaptive resolution textures suitable for photorealistic visualization
  and CAD export.

EXAMPLES:
  rux create texture                   # Default: mesh -> textured_mesh
  rux create texture -m room1 -o room1_tex  # Custom names
  rux create texture --debug-colors    # Distinct colors for UV debugging
  rux create texture --texels-per-meter 800  # Higher texture resolution
  rux -p scan.rux create texture       # Custom project path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames with RGB
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes && rooms        # Segment architecture
  4. rux create mesh                   # Generate 3D mesh
  5. rux create texture                # Apply textures to mesh
  6. rux export rhino textured.3dm     # Export textured model

NOTES:
  - Requires mesh and point cloud with RGB in project database
  - Mesh is automatically unwelded before texturing for PCL compatibility
  - Adaptive texture resolution based on polygon physical size
  - Output mesh includes embedded texture atlas
  - Default input: 'mesh', default output: 'textured_mesh'
)");

  sub->add_option("-m, --mesh-name", opt->mesh_name,
                  "Name of the input mesh in ProjectDB")
      ->default_val(opt->mesh_name);

  sub->add_option("-o, --output-name", opt->output_name,
                  "Name for the output textured mesh in ProjectDB")
      ->default_val(opt->output_name);

  sub->add_option("-c, --cloud-name", opt->cloud_name,
                  "Name of the point cloud in ProjectDB")
      ->default_val(opt->cloud_name);

  sub->add_flag("--debug-colors", opt->debug_colors,
                "Use distinct colors per polygon for UV mapping verification");

  sub->add_option("--texels-per-meter", opt->texels_per_meter,
                  "Target texture resolution in pixels per meter")
      ->default_val(opt->texels_per_meter);

  sub->add_option("--max-resolution", opt->max_resolution,
                  "Maximum texture size in pixels")
      ->default_val(opt->max_resolution);

  sub->add_option("--atlas-tile-size", opt->atlas_tile_size,
                  "Atlas tile size for PCL visualization")
      ->default_val(opt->atlas_tile_size);

  sub->add_option("--distance-threshold", opt->distance_threshold,
                  "Max distance from point to surface in meters")
      ->default_val(opt->distance_threshold);

  sub->add_option("--texture-dir", opt->texture_dir,
                  "Directory to stage generated texture images in "
                  "(default: a unique directory under the system temp "
                  "directory, removed once the textures are in the project)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_texture(*opt, *global_opt);
  });
}
int run_subcommand_texture(SubcommandTextureOptions const &opt,
                           const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Texturing mesh from project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation against the `texture` stage contract. Until #246
    // this command was the one `rux create` subcommand with no contract check
    // at all: `validate_texture_prerequisites()` existed but was never called,
    // so a missing sensor frame surfaced as a failure deep inside texturing.
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::texture,
            {{"mesh", opt.mesh_name}, {"cloud", opt.cloud_name}});
        rc != RuxError::SUCCESS)
      return rc;

    int logId = db.log_pipeline_start(
        "texture_mesh",
        fmt::format(
            R"({{"mesh_name":"{}","output_name":"{}","cloud_name":"{}","debug_colors":{}}})",
            opt.mesh_name, opt.output_name, opt.cloud_name, opt.debug_colors));

    spdlog::trace("Loading mesh '{}' from ProjectDB", opt.mesh_name);
    auto mesh = db.mesh(opt.mesh_name);

    spdlog::trace("Loading point cloud '{}' from ProjectDB", opt.cloud_name);
    auto cloud = db.point_cloud_xyzrgb(opt.cloud_name);
    spdlog::info("Loaded point cloud with {} points", cloud->size());

    // Optionally load normals
    CloudNConstPtr normals;
    std::string normals_name = opt.cloud_name + "_normals";
    if (db.has_point_cloud(normals_name)) {
      normals = db.point_cloud_normal(normals_name);
      spdlog::info("Loaded normals cloud '{}' ({} normals)", normals_name,
                   normals->size());
    }

    // Unweld mesh for PCL texture compatibility
    spdlog::info("Unwelding mesh ({} polygons)", mesh->polygons.size());
    auto unwelded = reusex::geometry::unweld_mesh(*mesh, 0.0f);
    spdlog::info("Unwelded mesh: {} polygons", unwelded->polygons.size());

    // Configure quality parameters
    reusex::geometry::TextureQualityParams quality;
    quality.texels_per_meter = opt.texels_per_meter;
    quality.max_resolution = opt.max_resolution;
    quality.atlas_tile_size = opt.atlas_tile_size;
    quality.distance_threshold = opt.distance_threshold;

    // Resolve the texture staging directory here rather than letting the
    // library do it implicitly, so this command can clean up after itself.
    // When --texture-dir is not given we own the (temp) directory and delete
    // it once the images are safely inside the project database; when the
    // user names a directory it is theirs and we never remove it (issue #245).
    const bool owns_staging_dir = opt.texture_dir.empty();
    quality.texture_dir =
        reusex::geometry::prepare_texture_dir(opt.texture_dir);
    spdlog::debug("Staging textures in {}", quality.texture_dir.string());

    /// Removes the staging directory on every exit path, but only when this
    /// command created it.
    struct StagingDirGuard {
      fs::path dir;
      bool owned;
      ~StagingDirGuard() {
        if (!owned)
          return;
        std::error_code ec;
        fs::remove_all(dir, ec);
        if (ec)
          spdlog::warn("Failed to remove texture staging directory {}: {}",
                       dir.string(), ec.message());
      }
    } staging_guard{quality.texture_dir, owns_staging_dir};

    spdlog::trace("Generating textured mesh");
    auto textured_mesh = reusex::geometry::texture_mesh_with_cloud(
        unwelded, cloud, normals, opt.debug_colors, quality);

    spdlog::trace("Saving textured mesh to ProjectDB as '{}'", opt.output_name);
    db.save_mesh(opt.output_name, *textured_mesh, "texture_mesh");

    spdlog::info("Textured mesh '{}' saved to ProjectDB", opt.output_name);

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Mesh texturing failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

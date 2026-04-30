// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "get/mesh.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <filesystem>

namespace {
/**
 * @brief Copy texture files and update TextureMesh material paths to be relative.
 *
 * PCL's loadOBJFile sets absolute paths to texture files. This function:
 * 1. Copies all texture images to the output directory
 * 2. Updates tex_materials to use relative paths (just filenames)
 *
 * @param mesh TextureMesh to update
 * @param output_dir Directory where OBJ/MTL files will be saved
 */
void prepare_texture_paths(pcl::TextureMesh& mesh, const fs::path& output_dir) {
  // Ensure output directory exists
  if (!fs::exists(output_dir)) {
    fs::create_directories(output_dir);
  }

  spdlog::info("Preparing {} texture(s) for export", mesh.tex_materials.size());

  for (size_t i = 0; i < mesh.tex_materials.size(); ++i) {
    auto& material = mesh.tex_materials[i];

    spdlog::debug("Material {}: name='{}', tex_file='{}'",
                  i, material.tex_name, material.tex_file);

    // Get the texture file path (might be absolute path to temp directory)
    fs::path texture_path(material.tex_file);

    if (texture_path.empty()) {
      spdlog::debug("Material {} has no texture file", i);
      continue;
    }

    if (!fs::exists(texture_path)) {
      spdlog::warn("Texture file not found: {}", material.tex_file);
      continue;
    }

    // Copy texture to output directory
    fs::path texture_filename = texture_path.filename();
    fs::path dest_path = output_dir / texture_filename;

    try {
      // Copy the texture file
      if (texture_path != dest_path) {
        fs::copy_file(texture_path, dest_path, fs::copy_options::overwrite_existing);
        spdlog::info("Copied texture: {}", texture_filename.string());
      }

      // Update material to use just the filename (relative path)
      material.tex_file = texture_filename.string();
      spdlog::debug("Updated material texture path to: {}", material.tex_file);

    } catch (const std::exception& e) {
      spdlog::error("Failed to copy texture {}: {}", texture_path.string(), e.what());
    }
  }
}
}  // anonymous namespace

int run_subcommand_get_mesh(SubcommandGetMeshOptions const &opt, const RuxOptions &global_opt) {
  spdlog::info("Exporting mesh '{}' from project: {}", opt.mesh_name,
               global_opt.project_db.string());

  try {
    // Open ProjectDB in read-only mode
    reusex::ProjectDB db(global_opt.project_db, /* readOnly */ true);

    // Check if mesh exists
    if (!db.has_mesh(opt.mesh_name)) {
      spdlog::error("Mesh '{}' not found in ProjectDB", opt.mesh_name);
      auto mesh_names = db.list_meshes();
      if (!mesh_names.empty()) {
        spdlog::info("Available meshes: {}", fmt::join(mesh_names, ", "));
      }
      return RuxError::INVALID_ARGUMENT;
    }

    // Get mesh format from database
    std::string db_format = db.mesh_format(opt.mesh_name);
    bool is_textured = (db_format == "obj_textured");
    bool is_ply = (db_format == "ply_binary");

    // Determine output format
    // Priority: 1) explicit --format flag, 2) output file extension, 3) database format
    std::string output_format;
    std::string file_extension;

    if (!opt.format.empty()) {
      // User explicitly set format
      output_format = opt.format;
    } else if (!opt.output_path.empty() && !opt.output_path.extension().empty()) {
      // Output path has extension
      file_extension = opt.output_path.extension().string();
      if (file_extension.size() > 1) {
        output_format = file_extension.substr(1);  // Remove leading '.'
      }
    } else {
      // Use database format
      if (is_textured) {
        output_format = "obj";
      } else if (is_ply) {
        output_format = "ply";
      } else {
        output_format = "ply";  // Default fallback
      }
    }

    // Convert format to lowercase
    std::transform(output_format.begin(), output_format.end(), output_format.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Determine output path
    fs::path output = opt.output_path;
    if (output.empty()) {
      output = fs::current_path() / (opt.mesh_name + "." + output_format);
    }

    spdlog::debug("Mesh database format: {}, output format: {}", db_format, output_format);

    // Load and export mesh
    if (is_textured) {
      auto texture_mesh = db.texture_mesh(opt.mesh_name);
      spdlog::info("Loaded textured mesh with {} materials",
                   texture_mesh->tex_materials.size());

      if (output_format == "obj") {
        // Prepare textures: copy to output directory and update paths to be relative
        fs::path output_dir = output.parent_path();
        if (output_dir.empty()) {
          output_dir = fs::current_path();
        }

        spdlog::debug("Preparing textures in directory: {}", output_dir.string());
        prepare_texture_paths(*texture_mesh, output_dir);

        // Now save the OBJ with updated relative paths
        if (pcl::io::saveOBJFile(output.string(), *texture_mesh) != 0) {
          spdlog::error("Failed to save textured mesh to OBJ file");
          return RuxError::IO;
        }
        spdlog::info("Textured mesh exported to: {}", output.string());

        // Check if .mtl and textures were created
        fs::path mtl_file = output.parent_path() / (output.stem().string() + ".mtl");
        if (fs::exists(mtl_file)) {
          spdlog::info("Material file created: {}", mtl_file.string());

          // Log texture files that were copied
          int texture_count = 0;
          for (const auto& material : texture_mesh->tex_materials) {
            if (!material.tex_file.empty()) {
              fs::path tex_path = output_dir / material.tex_file;
              if (fs::exists(tex_path)) {
                texture_count++;
                spdlog::debug("Texture file: {}", tex_path.string());
              }
            }
          }
          if (texture_count > 0) {
            spdlog::info("Copied {} texture file(s) to output directory", texture_count);
          }
        }
      } else if (output_format == "ply") {
        spdlog::warn(
            "Converting textured mesh to PLY - texture information will be lost");
        pcl::PolygonMesh poly_mesh;
        poly_mesh.cloud = texture_mesh->cloud;
        if (!texture_mesh->tex_polygons.empty()) {
          poly_mesh.polygons = texture_mesh->tex_polygons[0];
        }
        if (pcl::io::savePLYFileBinary(output.string(), poly_mesh) != 0) {
          spdlog::error("Failed to save mesh to PLY file");
          return RuxError::IO;
        }
        spdlog::info("Mesh exported to: {}", output.string());
      } else {
        spdlog::error("Unsupported format '{}' for textured mesh", output_format);
        spdlog::info("Textured meshes support: obj (with textures), ply (textures discarded)");
        return RuxError::INVALID_ARGUMENT;
      }
    } else {
      // Regular polygon mesh
      auto mesh = db.mesh(opt.mesh_name);
      spdlog::info("Loaded mesh with {} vertices, {} faces",
                   mesh->cloud.width * mesh->cloud.height,
                   mesh->polygons.size());

      if (output_format == "obj") {
        if (pcl::io::saveOBJFile(output.string(), *mesh) != 0) {
          spdlog::error("Failed to save mesh to OBJ file");
          return RuxError::IO;
        }
      } else if (output_format == "ply") {
        if (pcl::io::savePLYFileBinary(output.string(), *mesh) != 0) {
          spdlog::error("Failed to save mesh to PLY file");
          return RuxError::IO;
        }
      } else {
        spdlog::error("Unknown format: {}", output_format);
        spdlog::info("Supported formats: obj, ply");
        return RuxError::INVALID_ARGUMENT;
      }
      spdlog::info("Mesh exported to: {}", output.string());
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to export mesh: {}", e.what());
    return RuxError::IO;
  }
}

void setup_subcommand_get_mesh(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGetMeshOptions>();
  auto *sub = app.add_subcommand(
      "mesh", "Export a specific mesh from ProjectDB to file");

  sub->add_option("mesh_name", opt->mesh_name, "Name of mesh in ProjectDB")
      ->required();

  sub->add_option("-o,--output", opt->output_path,
                  "Output file path (default: ./{mesh_name}.{ext})");

  sub->add_option("-f,--format", opt->format,
                  "Output format override: obj, ply (default: auto-detect from database)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_get_mesh");
    return run_subcommand_get_mesh(*opt, *global_opt);
  });
}

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "ReUseX/geometry/texture_mesh.hpp"
#include "ReUseX/types.hpp"
#include "ReUseX/visualize/pcl.hpp"

#include <Eigen/Dense>
#include <fmt/format.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/surface/texture_mapping.h>
#include <range/v3/view/enumerate.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <spdlog/spdlog.h>

#include <spdmon/spdmon.hpp>

#include <filesystem>

namespace fs = std::filesystem;

namespace ReUseX::geometry {
pcl::TextureMesh::Ptr texture_mesh_with_cloud(pcl::PolygonMesh::Ptr mesh,
                                              CloudConstPtr cloud) {
  spdlog::trace("Entering ReUseX::geometry::texture_mesh_with_cloud");

  // Copy cloud and polygons
  pcl::TextureMesh::Ptr textured_mesh(new pcl::TextureMesh);
  textured_mesh->cloud = mesh->cloud;

  spdlog::trace("Converting mesh cloud to PointXYZ");
  CloudLocPtr cloud_xyz(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud_xyz);
  CloudNPtr normals(new CloudN);
  normals->resize(cloud_xyz->size());
  normals->width = cloud_xyz->width;
  normals->height = cloud_xyz->height;
  for (auto &p : normals->points)
    p.getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // INFO: Merge triangles into polygons
  // Assumption triangles making up polygons are consecutive in the list of
  // polygons
  Eigen::Vector3f prev_normal = Eigen::Vector3f::Zero();
  for (const auto &poly : mesh->polygons) {

    // TODO: Extract as utility function
    // Compute normal of the polygon
    Eigen::Vector3f f_normal = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < poly.vertices.size(); ++i) {
      auto idx_c = poly.vertices[i];
      auto idx_n = poly.vertices[(i + 1) % poly.vertices.size()];
      const auto &v_c = cloud_xyz->points[idx_c].getVector3fMap();
      const auto &v_n = cloud_xyz->points[idx_n].getVector3fMap();
      f_normal += v_c.cross(v_n);
    }
    f_normal.normalize();

    // Assign normal to all vertices in the polygon
    for (const auto &vertex : poly.vertices)
      normals->points[vertex].getNormalVector3fMap() += f_normal;

    // Start a new polygon
    if (f_normal.dot(prev_normal) < 0.95f) {
      textured_mesh->tex_polygons.push_back(std::vector<pcl::Vertices>());
      prev_normal = f_normal;
    }
    textured_mesh->tex_polygons.back().push_back(poly);
  }

  // Normalize all normals
  for (auto &p : normals->points)
    p.getNormalVector3fMap().normalize();

  spdlog::trace("Concatenateing fields normals to the mesh cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud_xyz, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, textured_mesh->cloud);

  spdlog::trace("Create directory for textures");
  static constexpr char texture_dir[] = "./mesh_textures";
  if (fs::exists(texture_dir)) {
    fs::remove_all(texture_dir);
  }
  fs::create_directory(texture_dir);
  fs::path base_path = fs::path(texture_dir);

  spdlog::trace("Resizing texture mesh structures");
  const size_t nr_polygons = textured_mesh->tex_polygons.size();
  textured_mesh->tex_materials.resize(nr_polygons);
  textured_mesh->tex_coordinates.resize(nr_polygons);
  textured_mesh->tex_coord_indices.resize(nr_polygons);
  for (size_t i = 0; i < textured_mesh->tex_polygons.size(); ++i)
    textured_mesh->tex_coord_indices[i].resize(
        textured_mesh->tex_polygons[i].size());

  spdlog::trace("Create materials");
  for (auto &&[idx, mat] :
       textured_mesh->tex_materials | ranges::views::enumerate) {

    auto name = fmt::format("texture-{:06}", idx);
    fs::path texture_path = base_path / fmt::format("{}.jpg", name);

    mat.tex_file = texture_path.string();
    mat.tex_name = name;

    mat.tex_Ka.r = 0.2f;
    mat.tex_Ka.g = 0.2f;
    mat.tex_Ka.b = 0.2f;

    mat.tex_Kd.r = 0.8f;
    mat.tex_Kd.g = 0.8f;
    mat.tex_Kd.b = 0.8f;

    mat.tex_Ks.r = 1.0f;
    mat.tex_Ks.g = 1.0f;
    mat.tex_Ks.b = 1.0f;

    mat.tex_d = 1.0f;
    mat.tex_Ns = 75.0f;
    mat.tex_illum = 2;
  }

  spdlog::trace("Create textutes form points in the cloud");
  {
    auto logger = spdmon::LoggerProgress("Createing material", nr_polygons);

    for (auto &&[idx, mat] :
         textured_mesh->tex_materials | ranges::views::enumerate) {

      const auto &polygons = textured_mesh->tex_polygons[idx];
      const Eigen::Vector3f normal{};
      const Eigen::Vector3f center{};

      // TODO: Get the size of the polygon

      cv::Mat image(1024, 1024, CV_8UC3, cv::Scalar(0, 0, 0));

      // TODO: Fill the image with the texture from the cloud points

      cv::imwrite(mat.tex_file, image);

      ++logger;
    }
  }

  spdlog::debug("Number of materials: {}", textured_mesh->tex_materials.size());
  spdlog::debug("Number of texture polygons: {}",
                textured_mesh->tex_polygons.size());
  spdlog::debug("Number of texture coordinates: {}",
                textured_mesh->tex_coordinates.size());
  for (auto [idx, poly_group] :
       textured_mesh->tex_polygons | ranges::views::enumerate) {
    spdlog::debug("  Number of polygons in group {}: {}", idx,
                  poly_group.size());
  }
  spdlog::trace("Nuber of coordinates {}",
                textured_mesh->tex_coord_indices.size());

  return textured_mesh;
}

pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes) {
  spdlog::trace("Entering ReUseX::geometry::texture_mesh");

  // Copy cloud and polygons
  pcl::TextureMesh::Ptr textured_mesh(new pcl::TextureMesh);
  textured_mesh->cloud = mesh->cloud;

  spdlog::trace("Converting mesh cloud to PointXYZ");
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  for (auto &p : normals->points)
    p.getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // INFO: Merge triangles into polygons
  // Assumption triangles making up polygons are consecutive in the list of
  // polygons
  textured_mesh->tex_polygons.resize(1);
  textured_mesh->tex_polygons[0] = mesh->polygons;

  for (const auto &poly : mesh->polygons) {

    // TODO: Extract as utility function
    // Compute normal of the polygon
    Eigen::Vector3f f_normal = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < poly.vertices.size(); ++i) {
      auto idx_c = poly.vertices[i];
      auto idx_n = poly.vertices[(i + 1) % poly.vertices.size()];
      const auto &v_c = cloud->points[idx_c].getVector3fMap();
      const auto &v_n = cloud->points[idx_n].getVector3fMap();
      f_normal += v_c.cross(v_n);
    }
    f_normal.normalize();

    // Assign normal to all vertices in the polygon
    for (const auto &vertex : poly.vertices)
      normals->points[vertex].getNormalVector3fMap() += f_normal;
  }

  // Normalize all normals
  for (auto &p : normals->points)
    p.getNormalVector3fMap().normalize();

  spdlog::trace("Concatenateing fields normals to the mesh cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, textured_mesh->cloud);

  spdlog::debug("Number of polygons: {}", textured_mesh->tex_polygons.size());
  size_t num_faces = 0;
  for (const auto &poly_group : textured_mesh->tex_polygons)
    num_faces += poly_group.size();
  spdlog::debug("Total number of faces: {}", num_faces);
  spdlog::debug("Number of vertices: {}", cloud->size());

  spdlog::trace("Create directory for textures");
  static constexpr char texture_dir[] = "./mesh_textures";
  if (fs::exists(texture_dir)) {
    fs::remove_all(texture_dir);
  }
  fs::create_directory(texture_dir);
  fs::path base_path = fs::path(texture_dir);

  spdlog::trace("Retrive all cameras and textures from the database");
  pcl::texture_mapping::CameraVector cameras{};
  cameras.resize(poses.size());
  {
    auto logger =
        spdmon::LoggerProgress("Retrieving textures and cameras", poses.size());

    // TODO: Get all the textures and cameras from the database
    for (auto &&[i, inner] : poses | ranges::views::enumerate) {
      auto &[id, pose] = inner;

      rtabmap::Signature node = nodes.find(id)->second;
      rtabmap::SensorData data = node.sensorData();
      data.uncompressData();

      cv::Mat image = data.imageRaw();
      if (image.empty()) {
        spdlog::warn("Empty image for node id {}", id);
        ++logger;
        continue;
      }
      auto name = fmt::format("texture-{:06}", i);
      fs::path texture_path = base_path / fmt::format("{}.jpg", name);
      cv::imwrite(texture_path.string(), image);

      cameras[i] = pcl::texture_mapping::Camera();
      cameras[i].focal_length_w = data.cameraModels().at(0).fx();
      cameras[i].focal_length_h = data.cameraModels().at(0).fy();
      cameras[i].center_w = data.cameraModels().at(0).cx();
      cameras[i].center_h = data.cameraModels().at(0).cy();
      cameras[i].width = static_cast<int>(image.cols);
      cameras[i].height = static_cast<int>(image.rows);
      cameras[i].pose = Eigen::Affine3f(pose.toEigen4f());
      cameras[i].texture_file = texture_path.string();

      pcl::TexMaterial material;
      material.tex_file = texture_path.string();
      material.tex_name = name;

      material.tex_Ka.r = 0.2f;
      material.tex_Ka.g = 0.2f;
      material.tex_Ka.b = 0.2f;

      material.tex_Kd.r = 0.8f;
      material.tex_Kd.g = 0.8f;
      material.tex_Kd.b = 0.8f;

      material.tex_Ks.r = 1.0f;
      material.tex_Ks.g = 1.0f;
      material.tex_Ks.b = 1.0f;

      material.tex_d = 1.0f;
      material.tex_Ns = 75.0f;
      material.tex_illum = 2;

      textured_mesh->tex_materials.push_back(material);
      ++logger;
    }
  }

  spdlog::debug("Number of cameras: {}", cameras.size());
  spdlog::debug("Number of materials: {}", textured_mesh->tex_materials.size());
  spdlog::debug("Number of texture polygons: {}",
                textured_mesh->tex_polygons.size());
  spdlog::debug("Number of texture coordinates: {}",
                textured_mesh->tex_coordinates.size());

  pcl::TextureMapping<pcl::PointXYZ> tm;
  tm.textureMeshwithMultipleCameras(*textured_mesh, cameras);

  if (textured_mesh->tex_polygons.size() >
      textured_mesh->tex_materials.size()) {
    pcl::TexMaterial material;
    material.tex_name = "texture-unassigned";
    material.tex_file = textured_mesh->tex_materials[0].tex_file;
    spdlog::warn("Number of texture polygons ({}) is greater than number of "
                 "materials ({}). Adding default material '{}' at {}.",
                 textured_mesh->tex_polygons.size(),
                 textured_mesh->tex_materials.size(), material.tex_name,
                 material.tex_file);
    textured_mesh->tex_materials.push_back(material);
  }

  spdlog::debug("Number of materials: {}", textured_mesh->tex_materials.size());
  spdlog::debug("Number of texture polygons: {}",
                textured_mesh->tex_polygons.size());
  spdlog::debug("Number of texture coordinates: {}",
                textured_mesh->tex_coordinates.size());
  for (auto [idx, poly_group] :
       textured_mesh->tex_polygons | ranges::views::enumerate) {
    spdlog::debug("  Number of polygons in group {}: {}", idx,
                  poly_group.size());
  }
  spdlog::trace("Nuber of coordinates {}",
                textured_mesh->tex_coord_indices.size());
  if (textured_mesh->tex_coord_indices.size() !=
      textured_mesh->tex_polygons.size()) {
    spdlog::warn("Number of texture coordinate indices ({}) does not match "
                 "number of texture polygons ({}).",
                 textured_mesh->tex_coord_indices.size(),
                 textured_mesh->tex_polygons.size());
    textured_mesh->tex_coord_indices.resize(textured_mesh->tex_polygons.size());
    for (size_t i = 0; i < textured_mesh->tex_polygons.size(); ++i) {
      textured_mesh->tex_coord_indices[i].resize(
          textured_mesh->tex_polygons[i].size());
    }
  }

  // pcl::visualization::PCLVisualizer::Ptr viewer(
  //     new pcl::visualization::PCLVisualizer("Textured Mesh Viewer"));
  // ReUseX::visualize::addCameraFrustums(viewer, cameras);
  // viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  //// viewer->addPolygonMesh(*mesh, "original mesh");
  //// viewer->setPointCloudRenderingProperties(
  ////    pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "original mesh");
  //// viewer->addTextureMesh(*textured_mesh, "textured mesh");
  // viewer->setBackgroundColor(0.1, 0.1, 0.1);
  // viewer->addCoordinateSystem(1.0f);
  // viewer->resetCamera();

  // while (!viewer->wasStopped()) {
  //   viewer->spinOnce(100);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }

  return textured_mesh;
}
} // namespace ReUseX::geometry

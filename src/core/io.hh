#pragma once
#define PCL_NO_PRECOMPILE

#include <filesystem>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

namespace ReUseX {

/// @brief Save the header to a file.
/// @param header
/// @param output_path
/// @return
static int save_header(fs::path const &output_path,
                       typename pcl::PCLHeader const &header) {

  fs::path p(output_path);
  p.replace_extension(".head");

  fs::exists(p) ? fs::remove(p) : fs::create_directories(p.parent_path());

  std::ofstream header_file(p);
  if (header_file.is_open()) {
    header_file << header.frame_id << std::endl;
    header_file << header.stamp << std::endl;
    header_file << header.seq << std::endl;
    header_file.close();
  }

  return 0;
}

/// @brief Load the header from a file.
/// @param filename
/// @return pcl::PCLHeader
static pcl::PCLHeader load_header(fs::path const &filename) {
  pcl::PCLHeader header;

  fs::path head(filename);
  head.replace_extension(".head");

  if (fs::exists(head)) {
    std::ifstream header_file(head);
    if (header_file.is_open()) {
      header_file >> header.frame_id;
      header_file >> header.stamp;
      header_file >> header.seq;
      header_file.close();
    }
  }
  return header;
}

/// @brief  Save a point cloud to disk.
/// @tparam T
/// @param data
/// @param output_path
/// @return
template <typename PointT>
static int save(fs::path const &output_path,
                typename pcl::PointCloud<PointT>::Ptr const &cloud) {

  auto resutl = pcl::io::savePCDFile(output_path, *cloud, true);

  save_header(output_path, cloud->header);

  return resutl;
}

/// @brief Load a point cloud from disk.
/// @tparam T
/// @param input_path
/// @return
template <class PointT>
static typename pcl::PointCloud<PointT>::Ptr load(fs::path const &input_path) {

  using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;

  CloudPtr cloud = CloudPtr(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(input_path, *cloud);

  cloud->header = load_header(input_path);

  return cloud;
}

#undef PCL_NO_PRECOMPILE

bool save(
    fs::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices);

bool read(fs::path const &input_path,
          std::vector<pcl::ModelCoefficients> &model_coefficients,
          std::vector<Eigen::Vector4f,
                      Eigen::aligned_allocator<Eigen::Vector4f>> &centroids,
          std::vector<std::shared_ptr<pcl::Indices>> &inlier_indices);
} // namespace ReUseX

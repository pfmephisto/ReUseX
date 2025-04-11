#define PCL_NO_PRECOMPILE

#include "slam.hh"

#include "../types/Geometry/PointCloud.hh"
#include "compute_normals.hh"
#include "fmt_formatter.hh"
#include "icp.hh"
#include "io.hh"
#include "progress_bar.hh"

#include <fmt/printf.h>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ReUseX {
int slam(fs::path const &directory) {

  auto duration = std::chrono::nanoseconds(0);

  fmt::print("SLAM: {}\n", directory.c_str());

  // Get the files & sort them
  std::vector<fs::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(directory))
    if (entry.is_regular_file() && entry.path().extension() == ".pcd")
      files.push_back(entry.path());
  std::sort(files.begin(), files.end());

  // Load the clouds
  std::vector<pcl::PointCloud<PointT>::Ptr,
              Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>>
      clouds;
  std::transform(files.begin(), files.end(), std::back_inserter(clouds),
                 [](const auto &file) { return load<PointT>(file); });

  // Calculate Registartion
  ///////////////////////////////////////////////////////////////////////////////
  auto transforms =
      std::vector<Eigen::Matrix4f>(clouds.size(), Eigen::Matrix4f::Identity());
  auto reg_bar = util::progress_bar(clouds.size(), "Registration");
  reg_bar.update(); // Skipping the first frame
#pragma omp parallel for shared(transforms)
  for (std::size_t i = 1; i < clouds.size(); ++i) {
    // ICP
    transforms[i] = pair_align<PointT>(clouds[i], clouds[i - 1], 0.1);
    // fmt::print("Matrix:\n{}\n", transforms.at(i));
    // fmt::print(transforms.at(i));
    reg_bar.update();
  }
  duration += reg_bar.stop();

  // Compute compound transformations
  ///////////////////////////////////////////////////////////////////////////////
  auto update_bar =
      util::progress_bar(clouds.size(), "Compute compound transformations");
  auto compund_transforms =
      std::vector<Eigen::Matrix4f>(clouds.size(), Eigen::Matrix4f::Identity());
  for (std::size_t i = 1; i < clouds.size(); ++i) {
    compund_transforms[i] = compund_transforms[i - 1] * transforms[i];
  }
  duration += update_bar.stop();

  // Update Position
  ///////////////////////////////////////////////////////////////////////////////
  auto move_bar = util::progress_bar(clouds.size(), "Moving clouds");
  move_bar.update(); // Skipping the first frame
#pragma omp parallel for shared(clouds)
  for (std::size_t i = 1; i < clouds.size(); ++i) {
    pcl::transformPointCloudWithNormals(*clouds[i], *clouds[i],
                                        compund_transforms[i]);
    move_bar.update();
  }
  duration += move_bar.stop();

  // for (size_t i = 0; i < clouds.size(); ++i)
  //   polyscope::display<pcl::PointCloud<PointT> const &>(
  //       *clouds[i],
  //       fmt::format("Cloud {}", clouds[i]->header.frame_id).c_str());

  // auto save_bar = util::progress_bar(clouds.size(),"Saving clouds");
  // #pragma omp parallel for shared(clouds, files)
  // for (std::size_t i = 0; i < clouds.size(); ++i){
  //     save<PointT>(files[i], clouds[i]);
  //     save_bar.update();
  // }
  // duration += save_bar.stop();

  fmt::print("Number of clouds: {} in {} secons.\n", clouds.size(),
             std::chrono::duration<double>(duration).count());

  return 0;
}
} // namespace ReUseX

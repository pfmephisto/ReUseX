#pragma once
#include <types/Data.hh>
#include <types/Dataset.hh>

#include <pcl/point_cloud.h>
// #include <pcl/point_representation.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/printf.h>

namespace ReUseX {

static Eigen::Matrix3d ipad_intrinsic() {
  Eigen::Matrix3d mat;
  mat << 212.14793333, 0.0, 127.12994667, 0.0, 212.14793333, 95.74299333, 0.0,
      0.0, 1.0;
  return mat;
}

/// @brief Parse a dataset
/// @param dataset The dataset to parse
/// @param output_path The path to save the parsed data
/// @param start The start frame
/// @param stop The stop frame
/// @param step The step size
void parse_Dataset(Dataset const &dataset, std::string const &output_path,
                   int start = 0,
                   std::optional<int> stop = std::optional<int>{},
                   int step = 5);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
CreateCloud(Data const &data,
            Eigen::Matrix3d intrinsic_matrix = ipad_intrinsic());

} // namespace ReUseX

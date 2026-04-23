// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "io/reusex.hpp"

#include <range/v3/action.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <fstream>
#include <map>
#include <set>

namespace reusex::io {

auto getPlanes(CloudLConstPtr planes, CloudNConstPtr normals,
               CloudLocConstPtr locations)
    -> std::tuple<EigenVectorContainer<double, 4>,
                  EigenVectorContainer<double, 3>, std::vector<IndicesPtr>> {
  EigenVectorContainer<double, 4> plane_coefficients{};
  EigenVectorContainer<double, 3> centroids{};
  std::vector<IndicesPtr> inlier_indices{};

  plane_coefficients.resize(normals->size());
  centroids.resize(locations->size());
  inlier_indices.resize(normals->size());

  reusex::debug("Number of planes: {}", planes->size());
  reusex::debug("Number of normals: {}", normals->size());
  reusex::debug("Number of locations: {}", locations->size());

  assert(normals->size() == locations->size() &&
         "Normals and locations size mismatch");

  for (size_t i = 0; i < normals->size(); ++i) {
    auto const &n = normals->points[i];
    auto const &c = locations->points[i];

    Eigen::Vector4d plane;
    plane.head<3>() = n.getNormalVector3fMap().cast<double>();
    plane[3] = -(plane[0] * c.x + plane[1] * c.y + plane[2] * c.z);

    plane_coefficients[i] = plane;
    centroids[i] = c.getVector3fMap().cast<double>();
    inlier_indices[i] = IndicesPtr(new Indices);
  }

  std::set<uint32_t> unique_labels{};
  for (const auto &pt : planes->points)
    unique_labels.insert(pt.label);

  // Remove the unlabeled points (label == 0)
  if (unique_labels.find(0) != unique_labels.end())
    unique_labels.erase(0);

  reusex::debug("Found {} unique plane labels: {}", unique_labels.size(),
                      fmt::join(unique_labels, ", "));

  std::unordered_map<uint32_t, size_t> label_to_index{};
  for (auto [idx, label] : unique_labels | ranges::views::enumerate)
    label_to_index[label] = idx;

  for (size_t i = 0; i < planes->size(); ++i) {
    auto const &label = planes->points[i].label;
    if (label == 0)
      continue;
    auto id = label_to_index[label];
    inlier_indices[id]->push_back(static_cast<int>(i));
  }

  return std::make_tuple(plane_coefficients, centroids, inlier_indices);
}

/**
 * @brief Save plane data to a custom file format.
 *
 * Saves plane model coefficients, inlier indices, and centroids to a text file
 * with the .planes extension.
 *
 * @param output_path Path to output file (extension will be changed to
 * .planes).
 * @param model_coefficients Plane model coefficients to save.
 * @param centroids Plane centroid positions.
 * @param inlier_indices Indices of points belonging to each plane.
 * @return True if save was successful.
 */
bool save(
    std::filesystem::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices) {

  std::filesystem::path p(output_path);
  p.replace_extension(".planes");

  std::filesystem::exists(p)
      ? std::filesystem::remove(p)
      : std::filesystem::create_directories(p.parent_path());

  const char *version = "# ReUseX file v0.0.1\n";
  const char *header =
      "# N coefficients c_1 ... c_n N Indices i_1 ... i_n Centroid(x y z)\n";

  assert(model_coefficients.size() == centroids.size());
  assert(model_coefficients.size() == inlier_indices.size());

  std::ofstream planes_file(p);
  if (planes_file.is_open()) {

    planes_file << version;
    planes_file << header;

    for (size_t i = 0; i < model_coefficients.size(); ++i) {

      planes_file << model_coefficients[i].values.size() << " ";
      for (const auto &value : model_coefficients[i].values)
        planes_file << value << " ";

      planes_file << inlier_indices[i]->size() << " ";
      for (const auto &index : *inlier_indices[i])
        planes_file << index << " ";

      planes_file << centroids[i].x() << " " << centroids[i].y() << " "
                  << centroids[i].z();

      planes_file << std::endl;
    }
    planes_file.close();
  }

  return true;
}

/**
 * @brief Read plane data from a custom file format.
 *
 * Reads plane model coefficients, inlier indices, and centroids from a
 * text file with the .planes format.
 *
 * @param input_path Path to input .planes file.
 * @param model_coefficients Output plane model coefficients.
 * @param centroids Output plane centroid positions.
 * @param inlier_indices Output indices of points belonging to each plane.
 * @return True if read was successful.
 */
bool read(std::filesystem::path const &input_path,
          std::vector<pcl::ModelCoefficients> &model_coefficients,
          std::vector<Eigen::Vector4f,
                      Eigen::aligned_allocator<Eigen::Vector4f>> &centroids,
          std::vector<std::shared_ptr<pcl::Indices>> &inlier_indices) {

  std::ifstream planes_file(input_path);
  for (std::string line; std::getline(planes_file, line);) {
    if (line.empty() || line[0] == '#')
      continue;

    std::istringstream iss(line);
    size_t num_coefficients = 0;
    iss >> num_coefficients;

    pcl::ModelCoefficients coefficients;
    coefficients.values.resize(num_coefficients);
    for (size_t i = 0; i < num_coefficients; ++i)
      iss >> coefficients.values[i];
    model_coefficients.push_back(coefficients);

    size_t num_indices = 0;
    iss >> num_indices;

    std::shared_ptr<pcl::Indices> indices(new pcl::Indices);
    indices->resize(num_indices);
    for (size_t i = 0; i < num_indices; ++i)
      iss >> indices->at(i);
    inlier_indices.push_back(indices);

    Eigen::Vector4f centroid;
    iss >> centroid.x();
    iss >> centroid.y();
    iss >> centroid.z();
    centroid.w() = 0.0f;
    centroids.push_back(centroid);
  }

  return true;
}
} // namespace reusex::io

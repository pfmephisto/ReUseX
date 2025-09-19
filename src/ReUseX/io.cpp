// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/io.hpp"

#include <fstream>

bool ReUseX::save(
    fs::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices) {

  fs::path p(output_path);
  p.replace_extension(".planes");

  fs::exists(p) ? fs::remove(p) : fs::create_directories(p.parent_path());

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

bool ReUseX::read(
    fs::path const &input_path,
    std::vector<pcl::ModelCoefficients> &model_coefficients,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
        &centroids,
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

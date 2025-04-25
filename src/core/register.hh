#pragma once
#include "types/Data.hh"
#include "types/Dataset.hh"
#include "types/Geometry/PointCloud.hh"

#include <Eigen/Core>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace ReUseX {

// void register_consecutive_edges(
//     std::unique_ptr<g2o::SparseOptimizer> &optimizer, Dataset &dataset,
//     std::vector<size_t> &indices, const size_t group_size = 5,
//     const double maxCorrespondence = 0.5,
//     const Eigen::Matrix<double, 6, 6> information_matrix =
//         Eigen::Matrix<double, 6, 6>::Identity());
//
// void register_nodes(std::unique_ptr<g2o::SparseOptimizer> &optimizer,
//                    Dataset &dataset, std::vector<size_t> &indices);

fs::path write_graph(fs::path path, Dataset &dataset,
                     std::vector<size_t> &indices, const size_t group_size,
                     const std::string solverName, // "lm_fix6_3_csparse"
                     const std::string kernelName, // "Huber"
                     const int maxIterations, const double maxCorrespondence,
                     const double deltaValue,
                     const Eigen::Matrix<double, 6, 6> information_matrix);

} // namespace ReUseX

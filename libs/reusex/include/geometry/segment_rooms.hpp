// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"
#include "reusex/io/reusex.hpp"
#include "reusex/types.hpp"
#include "reusex/utils/fmt_formatter.hpp"

#include <pcl/markov_clustering.hpp>

// GraphBLAS imports complex.h which defines a macro named 'I' that conflicts
// with the type in CLI11
#ifdef I
#undef I
#endif

#include <fmt/format.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <atomic>

namespace ReUseX::geometry {
struct SegmentRoomsOptions {
  IndicesConstPtr filter = nullptr; // Optional filter to limit processing

  float grid_size = 0.5F;
  float inflation = 2.0F;
  int expansion = 2;
  float pruning_threshold = 0.0001F;
  float convergence_threshold = 1e-8F;
  int max_iter = 100;

  // Optional cancellation flag. Caller retains ownership and must keep this
  // alive for the full duration of the segment_rooms(...) call.
  const std::atomic_bool *cancel_token = nullptr;
};

auto segment_rooms_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                        CloudLConstPtr planes,
                        const SegmentRoomsOptions &options) -> CloudLPtr;
auto segment_rooms(CloudConstPtr cloud, CloudNConstPtr normals,
                   CloudLConstPtr planes,
                   const SegmentRoomsOptions &options = SegmentRoomsOptions{})
    -> CloudLPtr;

} // namespace ReUseX::geometry

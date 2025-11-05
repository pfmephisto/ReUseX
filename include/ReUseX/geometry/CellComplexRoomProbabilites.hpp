// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "spdmon/spdmon.hpp"

#include <embree4/rtcore.h>

#include <pcl/filters/uniform_sampling.h>

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/ranges.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

// Spherical Fibonacci
static std::vector<Eigen::Vector3d> sampleSphericalFibonacci(size_t N) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(N);
  if (N == 0)
    return out;
  constexpr double golden_angle =
      M_PI * (3.0 - std::sqrt(5.0)); // ~2.3999632297

  for (size_t i = 0; i < N; ++i) {
    double t = (double)i;
    double z = 1.0 - 2.0 * t / (double)(N - 1); // maps to [1, -1] if N>1
    double phi = golden_angle * t;
    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    double x = std::cos(phi) * r;
    double y = std::sin(phi) * r;
    out.emplace_back(x, y, z);
  }
  return out;
}

namespace ReUseX::geometry {
template <typename PointT, typename PointN, typename PointL>
auto CellComplex::compute_room_probabilities(
    pcl::PointCloud<PointT>::ConstPtr cloud_,
    pcl::PointCloud<PointN>::ConstPtr normals_,
    pcl::PointCloud<PointL>::ConstPtr labels_, const double grid_size) -> void {

  using RTCVertex = struct RTCVertex {
    float x, y, z, r; // x, y, z coordinates and radius
  };

  using RTCNormal = struct RTCNormal {
    float x, y, z; // x, y, z components of the normal vector
  };

  using RTCData = struct RTCData {
    size_t label_index;
  };

  spdlog::trace("calling compute_room_probabilities");

  assert(cloud_->size() == labels_->size() &&
         "Point cloud and label cloud must have the same size");
  assert(cloud_->size() == normals_->size() &&
         "Point cloud and normal cloud must have the same size");

  const double radius_ = grid_size * M_SQRT2;

  spdlog::debug(
      "Using ray tracing for {} points with grid size {:.3} and radius {:.3}",
      cloud_->size(), grid_size, radius_);

  auto c_rp = this->add_property_map<Vertex, std::vector<double>>(
                      "c:room_probabilities")
                  .first;

  // Get unique labels
  std::vector<unsigned int> labels;
  labels.reserve(labels_->points.size());

  for (const auto &p : labels_->points)
    if (p.label != static_cast<unsigned int>(-1))
      labels.push_back(p.label);

  std::sort(labels.begin(), labels.end());
  labels.erase(std::unique(labels.begin(), labels.end()), labels.end());
  spdlog::debug("Room labels: {}", fmt::join(labels, ", "));

  for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit)
    c_rp[*cit] = std::vector<double>(labels.size() + 1, 0.0);
  this->n_rooms = labels.size();
  spdlog::debug("Number of rooms (labels): {}", labels.size());

  unsigned int old_mxcsr = _mm_getcsr(); // save current flagsq
  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  spdlog::trace("Creating device and scene for ray tracing");
  RTCDevice device_ = rtcNewDevice("verbose=0"); // 0-3
  RTCScene scene_ = rtcNewScene(device_);
  rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
  assert(device_ != nullptr && "Error creating Embree device");
  assert(scene_ != nullptr && "Error creating Embree scene");
  rtcSetDeviceErrorFunction(
      device_,
      [](void *, RTCError err, const char *str) {
        spdlog::error("Embree error {}:", str);
        switch (err) {
        case RTC_ERROR_NONE:
          break;
        case RTC_ERROR_UNKNOWN:
          throw std::runtime_error("Embree: An unknown error has occurred.");
          break;
        case RTC_ERROR_INVALID_ARGUMENT:
          throw std::runtime_error(
              "Embree: An invalid argument was specified.");
          break;
        case RTC_ERROR_INVALID_OPERATION:
          throw std::runtime_error(
              "Embree: The operation is not allowed for the specified object.");
          break;
        case RTC_ERROR_OUT_OF_MEMORY:
          throw std::runtime_error("Embree: There is not enough memory left to "
                                   "complete the operation.");
          break;
        case RTC_ERROR_UNSUPPORTED_CPU:
          throw std::runtime_error(
              "Embree: The CPU is not supported as it does not support SSE2.");
          break;
        case RTC_ERROR_CANCELLED:
          throw std::runtime_error(
              "Embree: The operation got cancelled by an Memory Monitor "
              "Callback or Progress Monitor Callback function.");
          break;
        default:
          throw std::runtime_error("Embree: An invalid error has occurred.");
          break;
        }
      },
      nullptr);

  // Intersect with ground plane to get line/ INFO: Create Embree scene
  spdlog::trace("Creating scene geometry for ray tracing");

  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud_);
  us.setRadiusSearch(grid_size);

  using RTCDataPtr = std::shared_ptr<RTCData>;
  std::vector<RTCDataPtr> rtc_data_vec{}; // To keep data alive
  for (size_t i = 0; i < labels.size(); ++i) {

    pcl::IndicesPtr indices(new pcl::Indices);
    for (size_t j = 0; j < cloud_->size(); ++j)
      if (labels_->points[j].label == labels[i])
        indices->push_back(static_cast<int>(j));

    spdlog::trace("Room label {}: {} points", labels[i], indices->size());

    // size_t n_points_before = indices->size();
    us.setIndices(indices);
    us.filter(*indices);
    // spdlog::trace(
    //     "Label {}: Reduced from {} to {} points after uniform sampling",
    //     labels[i], n_points_before, indices->size());

    RTCGeometry geometry_ =
        rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    RTCVertex *vb = (RTCVertex *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4,
        sizeof(RTCVertex), indices->size());

    RTCNormal *nb = (RTCNormal *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3,
        sizeof(RTCNormal), indices->size());

    for (size_t i = 0; i < indices->size(); ++i) {
      const auto &p = cloud_->points[indices->at(i)];
      const auto &n = normals_->points[indices->at(i)];

      vb[i].x = static_cast<float>(p.x);
      vb[i].y = static_cast<float>(p.y);
      vb[i].z = static_cast<float>(p.z);
      vb[i].r = static_cast<float>(radius_) * 1.05f;

      nb[i].x = static_cast<float>(n.normal_x);
      nb[i].y = static_cast<float>(n.normal_y);
      nb[i].z = static_cast<float>(n.normal_z);
    }

    rtc_data_vec.emplace_back(new RTCData{i});
    rtcSetGeometryUserData(geometry_, rtc_data_vec.back().get());

    rtcCommitGeometry(geometry_);
    rtcAttachGeometry(scene_, geometry_);
    rtcReleaseGeometry(geometry_);
  }

  spdlog::trace("Committing scene");
  rtcCommitScene(scene_);

  {
    const auto dirs = sampleSphericalFibonacci(100);
    //
    auto logger = spdmon::LoggerProgress("Computing room probabilities",
                                         this->num_cells());

#pragma omp parallel for schedule(dynamic)
    for (size_t cell_idx = 0; cell_idx < this->num_cells(); ++cell_idx) {
      auto cit = this->cells_begin();
      std::advance(cit, cell_idx);
      // const size_t idx = (*this)[*cit].id;
      // For each cell create n random rays
      // Count the number of intersections per id nad normalize

      std::vector<double> local_accum(c_rp[*cit].size(), 0.0);
      double *accum_ptr = local_accum.data();
      const int N = static_cast<int>(c_rp[*cit].size());

#pragma omp parallel for reduction(+ : accum_ptr[ : N])
      for (size_t i = 0; i < dirs.size(); ++i) {
        const auto dir = dirs[i];
        auto c = (*this)[*cit].pos;

        RTCRayHit rayhit;
        rayhit.ray.org_x = static_cast<float>(c.x());
        rayhit.ray.org_y = static_cast<float>(c.y());
        rayhit.ray.org_z = static_cast<float>(c.z());
        rayhit.ray.dir_x = static_cast<float>(dir.x());
        rayhit.ray.dir_y = static_cast<float>(dir.y());
        rayhit.ray.dir_z = static_cast<float>(dir.z());

        rayhit.ray.tnear = 0.001f; // Start a bit away from the origin
        rayhit.ray.tfar = std::numeric_limits<float>::infinity();
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        rtcIntersect1(scene_, &rayhit);

        if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) { // No hit
          accum_ptr[0] += 1;
          // spdlog::trace(fmt::format(fmt::fg(fmt::color::red), "No hit"));
          continue;
        }

        // Check if backside
        const auto normal =
            Eigen::Vector3f(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z)
                .normalized();
        const Eigen::Vector3f dir_vec(dir.x(), dir.y(), dir.z());
        if (normal.dot(dir_vec) > 0) {
          accum_ptr[0] += 1;
          // spdlog::trace(
          //     fmt::format(fmt::fg(fmt::color::yellow), "Backside hit"));
          continue; // Backside
        }
        // spdlog::trace(fmt::format(fmt::fg(fmt::color::green), "Frontside
        // hit"));

        auto geometry = rtcGetGeometry(scene_, rayhit.hit.geomID);
        rtcGetGeometryUserData(geometry);
        RTCData *data = (RTCData *)rtcGetGeometryUserData(geometry);

        // const auto label = labels[data->label_index];
        // spdlog::trace("Cell {:>3} ray {} hit label {} (index {})",
        //               (*this)[*cit].id, i, label, data->label_index);

        //  #pragma omp critical
        accum_ptr[data->label_index + 1] += 1;
        // spdlog::trace("Cell {:>3} hit label {} (index {})", idx,
        //               labels[data->label_index], data->label_index);
      }

      // Compute probabilities
      // double sum = std::accumulate(c_rp[*cit].begin(), c_rp[*cit].end(),
      // 0.0); spdlog::trace("Cell {:>3} sum = {:.3f}", idx, sum); if (sum > 0)
      //  for (size_t j = 0; j < c_rp[*cit].size(); ++j)
      //    c_rp[*cit][j] /= sum;
      //
      for (auto &p : local_accum)
        p /= dirs.size();
      c_rp[*cit] = std::move(local_accum);

      ++logger;
    }
  }
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);

  _mm_setcsr(old_mxcsr); // restore old flags

  // Log sum of probabilities
  auto sum_results = std::vector<double>(labels.size() + 1, 0.0);
  for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit)
    for (size_t i = 0; i < c_rp[*cit].size(); ++i)
      sum_results[i] += c_rp[*cit][i];
  spdlog::trace("Sum probabilities => [{:.3f}]", fmt::join(sum_results, ", "));
}
} // namespace ReUseX::geometry

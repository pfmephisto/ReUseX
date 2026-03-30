// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <pcl/PointIndices.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/geometry.h>
#include <pcl/common/utils.h>
#include <pcl/filters/filter.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
// #include <CGAL/Polygon_mesh_processing/connected_components.h>
// #include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>
// #include <CGAL/Polygon_mesh_processing/measure.h>
// #include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>
// #include <CGAL/Polygon_mesh_processing/orientation.h>
// #include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
// #include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
// #include <CGAL/Polygon_mesh_processing/repair.h>
// #include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
// #include <CGAL/Polygon_mesh_processing/stitch_borders.h>
// #include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

// Define USE_CUOPT, USE_HIGHS, CGAL_USE_SCIP, or CGAL_USE_GLPK in
// CMakeLists.txt to select MIP solver

#ifdef USE_CUOPT
#include <CGAL/cuOpt_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::cuOpt_mixed_integer_program_traits<double>;
#elif defined(USE_HIGHS)
#include <CGAL/HiGHS_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::HiGHS_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_SCIP)
#include <CGAL/SCIP_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_GLPK)
#include <CGAL/GLPK_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::GLPK_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_GUROBI)
#include "reusex/core/GUROBI_mixed_integer_program_traits.hh"
using MIP_Solver = CGAL::GUROBI_mixed_integer_program_traits<double>;
#endif

#include <cmath>
#include <queue>

// namespace PMP = CGAL::Polygon_mesh_processing;
namespace pcl {

template <typename PointT, typename PointNT = pcl::Normal,
          typename PointLT = pcl::Label>
class PCL_EXPORTS PolygonalSurfaceReconstruction : public PCLBase<PointT> {
    public:
  using Ptr =
      std::shared_ptr<PolygonalSurfaceReconstruction<PointT, PointNT, PointLT>>;
  using ConstPtr = std::shared_ptr<
      const PolygonalSurfaceReconstruction<PointT, PointNT, PointLT>>;

  using Indices = pcl::Indices;
  using IndicesPtr = pcl::IndicesPtr;
  using IndicesConstPtr = pcl::IndicesConstPtr;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  using PointCloudN = pcl::PointCloud<PointNT>;
  using PointCloudNPtr = typename PointCloudN::Ptr;
  using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

  using PointCloudL = pcl::PointCloud<PointLT>;
  using PointCloudLPtr = typename PointCloudL::Ptr;
  using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

    protected:
  using PCLBase<PointT>::indices_;
  using PCLBase<PointT>::deinitCompute;

  using PCLBase<PointT>::fake_indices_;

  using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
  using Point = typename Kernel::Point_3;
  using Vector = typename Kernel::Vector_3;

  using SurfaceMesh = CGAL::Surface_mesh<Point>;
  using Polygonal_surface_reconstruction =
      CGAL::Polygonal_surface_reconstruction<Kernel>;

    public:
  /** \brief Constructor for OrganizedMultiPlaneSegmentation. */
  PolygonalSurfaceReconstruction() = default;

  /** \brief Destructor for OrganizedMultiPlaneSegmentation. */
  ~PolygonalSurfaceReconstruction() override = default;

  /** \brief Provide a pointer to the input point cloud.
   * \param[in] cloud the input point cloud
   */
  inline void setInputCloud(const PointCloudPtr &cloud) { input_ = cloud; }

  /** \brief Get the input point cloud.
   * \return a pointer to the input point cloud
   */
  inline PointCloudPtr getInputCloud() const { return (input_); }

  /** \brief Provide a pointer to the input normals.
   * \param[in] normals the input normal cloud
   */
  inline void setInputNormals(const PointCloudNPtr &normals) {
    normals_ = normals;
  }

  /** \brief Get the input normals. */
  inline PointCloudNPtr getInputNormals() const { return (normals_); }

  /** \brief Provide a pointer to the input labels.
   * \param[in] labels the input label cloud
   */
  inline void setInputLabels(const PointCloudLPtr &labels) { labels_ = labels; }

  /** \brief Get the input labels. */
  inline PointCloudLPtr getInputLabels() const { return (labels_); }

  /** \brief Set the fitting parameter for polygonal surface reconstruction.
   * \param[in] fitting the fitting parameter
   */
  inline void setFittingParam(double fitting) { fitting_ = fitting; }

  /** \brief Get the fitting parameter for polygonal surface reconstruction.
   * \return the fitting parameter
   */
  inline double getFittingParam() const { return (fitting_); }

  /** \brief Set the coverage parameter for polygonal surface reconstruction.
   * \param[in] coverage the coverage parameter
   */
  inline void setCoverageParam(double coverage) { coverage_ = coverage; }

  /** \brief Get the coverage parameter for polygonal surface reconstruction.
   * \return the coverage parameter
   */
  inline double getCoverageParam() const { return (coverage_); }

  /** \brief Set the complexity parameter for polygonal surface reconstruction.
   * \param[in] complexity the complexity parameter
   */
  inline void setComplexityParam(double complexity) {
    complexity_ = complexity;
  }

  /** \brief Get the complexity parameter for polygonal surface reconstruction.
   * \return the complexity parameter
   */
  inline double getComplexityParam() const { return (complexity_); }

  // TODO: Add comprehensive documentation for segment() method
  // category=Documentation estimate=30m
  // Missing Doxygen documentation describing:
  // \param[out] output - The reconstructed polygonal mesh surface
  // \brief Performs polygonal surface reconstruction using CGAL optimization
  // Converts labeled planar regions into watertight polygonal mesh using
  // mixed-integer programming for optimal surface selection
  /** \brief    */
  void segment(pcl::PolygonMesh &output) {

    spdlog::trace("Solid Model Construction: segment() called");
    if (!initCompute()) {
      deinitCompute();
      return;
    }

    // TODO: Implement plane membership filtering for input point indices
    // category=Geometry estimate=3h
    // Currently processes all input indices without filtering. Should add:
    // 1. Check if points belong to detected planar regions (via label_map_)
    // 2. Optionally filter out points not assigned to any plane
    // 3. Add threshold parameter for minimum points per plane
    // Reduces noise in reconstruction and improves mesh quality

    Polygonal_surface_reconstruction algo(*indices_, *point_map_, *normal_map_,
                                          *label_map_);

    SurfaceMesh model;
    if (!algo.reconstruct<MIP_Solver>(model, fitting_, coverage_,

                                      complexity_)) {
      PCL_ERROR("[pcl::%s::segment] Reconstruction failed.\n",
                getClassName().c_str());
      deinitCompute();
      return;
    }

    toPolygonMesh(model, output);

    deinitCompute();
  }

    protected:
  /** \brief Initialize the segmentation process.
   * \return true if initialization was successful, false otherwise
   */
  bool initCompute() {

    // Check if input was set
    if (!input_) {
      PCL_ERROR("[initCompute] No input set.\n");
      return (false);
    }

    // If no point indices have been given, construct a set of indices for the
    // entire input point cloud
    if (!indices_) {
      fake_indices_ = true;
      indices_.reset(new Indices);
    }

    // If we have a set of fake indices, but they do not match the number of
    // points in the cloud, update them
    if (fake_indices_ && indices_->size() != input_->size()) {
      const auto indices_size = indices_->size();
      try {
        indices_->resize(input_->size());
      } catch (const std::bad_alloc &) {
        PCL_ERROR("[initCompute] Failed to allocate %lu indices.\n",
                  input_->size());
        return (false);
      }
      for (auto i = indices_size; i < indices_->size(); ++i) {
        (*indices_)[i] = i;
      }
    }

    // if (!PCLBase<PointT>::initCompute()) {
    //   PCL_ERROR("[pcl::%s::initCompute] Init failed.\n",
    //             getClassName().c_str());
    //   deinitCompute();
    //   return (false);
    // }

    // Check that the normals are present
    if (!normals_) {
      PCL_ERROR("[pcl::%s::segment] Must specify normals.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

    // Check that we got the same number of points and normals
    if (normals_->size() != input_->size()) {
      PCL_ERROR("[pcl::%s::segment] Number of points in input cloud "
                "(%zu) and normal "
                "cloud (%zu) do not match!\n",
                getClassName().c_str(),
                static_cast<std::size_t>(input_->size()),
                static_cast<std::size_t>(normals_->size()));
      deinitCompute();
      return (false);
    }

    // Check if normals contain NaN values and issue warning.
    for (const auto &point : normals_->points) {
      if (std::isnan(point.normal_x) || std::isnan(point.normal_y) ||
          std::isnan(point.normal_z)) {
        PCL_WARN("[pcl::%s::initCompute] Input normals contain NaN values.\n",
                 getClassName().c_str());
        break;
      }
    }

    point_map_.reset(new PointMap(input_));
    normal_map_.reset(new NormalMap(normals_));
    label_map_.reset(new LabelMap(labels_));

    return (true);
  }

  struct PointMap {

    PointMap(PointCloudPtr input) : input_(input) {}

    const Point &operator[](const int &idx) const {
      return Point(input_->points[idx].x, input_->points[idx].y,
                   input_->points[idx].z);
    }

    friend Point get(const PointMap &pm, const int &idx) {
      return Point(pm.input_->points[idx].x, pm.input_->points[idx].y,
                   pm.input_->points[idx].z);
    }
    friend void put(const PointMap &pm, int &idx, const Point &v) {
      pm.input_->points[idx].x = CGAL::to_double(v[0]);
      pm.input_->points[idx].y = CGAL::to_double(v[1]);
      pm.input_->points[idx].z = CGAL::to_double(v[2]);
    }

      private:
    PointCloudPtr input_ = {};
  };

  struct NormalMap {

    NormalMap(PointCloudNPtr normals) : normals_(normals) {}

    const Vector &operator[](const int &idx) const {
      return Vector(normals_->points[idx].normal_x,
                    normals_->points[idx].normal_y,
                    normals_->points[idx].normal_z);
    }

    friend Vector get(const NormalMap &nm, const int &idx) {
      return Vector(nm.normals_->points[idx].normal_x,
                    nm.normals_->points[idx].normal_y,
                    nm.normals_->points[idx].normal_z);
    }
    friend void put(const NormalMap &nm, int &idx, const Vector &v) {
      nm.normals_->points[idx].normal_x = CGAL::to_double(v[0]);
      nm.normals_->points[idx].normal_y = CGAL::to_double(v[1]);
      nm.normals_->points[idx].normal_z = CGAL::to_double(v[2]);
    }

      private:
    PointCloudNPtr normals_ = {};
  };

  struct LabelMap {

    LabelMap(PointCloudLPtr labels) : labels_(labels) {}

    const int &operator[](const int &idx) const {
      return labels_->points[idx].label - 1; // Adjusting to 0-based index;
    }
    friend int get(const LabelMap &lm, const int &idx) {
      return lm.labels_->points[idx].label - 1; // Adjusting to 0-based index;
    }
    friend void put(const LabelMap &lm, int &idx, const int &v) {
      lm.labels_->points[idx].label =
          (unsigned)v + 1; // Adjusting to 1-based index;
    }

      private:
    PointCloudLPtr labels_ = {};
  };

  static void toPolygonMesh(SurfaceMesh &model, pcl::PolygonMesh &mesh) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(
        new pcl::PointCloud<pcl::PointXYZ>);
    vertices->points.resize(model.number_of_vertices());
    for (const auto &v : model.vertices()) {
      const auto &point = model.point(v);
      vertices->points[v.idx()].x = CGAL::to_double(point.x());
      vertices->points[v.idx()].y = CGAL::to_double(point.y());
      vertices->points[v.idx()].z = CGAL::to_double(point.z());
    }
    pcl::toPCLPointCloud2(*vertices, mesh.cloud);
    mesh.polygons.resize(model.number_of_faces());
    for (const auto &f : model.faces()) {
      std::vector<uint32_t> polygon;
      auto h = model.halfedge(f); // get a halfedge of the face
      auto done = h;
      do {
        auto v = model.target(h);
        mesh.polygons[f.idx()].vertices.push_back(v.idx());
        h = model.next(h);
      } while (h != done);
    }
  }

  /** \brief Get a string representation of the name of this class. */
  inline const std::string getClassName() const {
    return ("PolygonalSurfaceReconstruction");
  }

    protected:
  /** \brief A pointer to the input point cloud */
  PointCloudPtr input_{nullptr};

  /** \brief A pointer to the input normals */
  PointCloudNPtr normals_{nullptr};

  /** \brief A pointer to the input labels */
  PointCloudLPtr labels_{nullptr};

  double fitting_ = 0.20;
  double coverage_ = 0.10;
  double complexity_ = 0.70;

  std::shared_ptr<PointMap> point_map_ = {};
  std::shared_ptr<NormalMap> normal_map_ = {};
  std::shared_ptr<LabelMap> label_map_ = {};

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

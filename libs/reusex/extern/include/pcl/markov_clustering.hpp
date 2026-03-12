// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: MIT

#pragma once
#include <cmath>

#include <pcl/PointIndices.h>
#include <pcl/Vertices.h>
#include <pcl/common/pca.h>
#include <pcl/common/utils.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <embree4/rtcore.h>

#include <LAGraph.h>
#include <LAGraphX.h>

#include "pmmintrin.h"
#include "xmmintrin.h" // for SSE intrinsics reytracing

// Optional: Include spdmon for progress logging and opencv for visual debugging
#include "spdmon/spdmon.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <thread>

#define USE_CLI 1

using IT = int32_t; // Index type
using NT = float;   // Numeric type

// Define OpenMP reduction for std::vector<std::tuple<int64_t, int64_t,
// double>>
#pragma omp declare reduction(                                                 \
        mergeTriplets : std::vector<std::tuple<IT, IT, NT>> : omp_out.insert(  \
                omp_out.end(), omp_in.begin(), omp_in.end()))                  \
    initializer(omp_priv = std::vector<std::tuple<IT, IT, NT>>())

// Only during CMAKE_BUILD_TYPE=Debug, CHECK will check the return value
// #ifdef NDEBUG
#if 1
#define LAGRAPH_CATCH(status)                                                  \
  {                                                                            \
    spdlog::error("LAGraph error {}: {} at \n {}:{}", (int)status, msg,        \
                  __FILE__, __LINE__);                                         \
  }

#define GRB_CATCH(status)                                                      \
  {                                                                            \
    spdlog::error("GraphBLAS error {} at \n {}:{}", (int)status, __FILE__,     \
                  __LINE__);                                                   \
  }
#else
#define LAGRAPH_CATCH(info) (info)
#define GRB_CATCH(info) (info)
#endif

namespace pcl {

/** \brief */

template <typename PointT, typename PointNT = pcl::Normal,
          typename PointLT = pcl::Label>
class PCL_EXPORTS MarkovClustering : public PCLBase<PointT> {
    public:
  using Ptr = std::shared_ptr<MarkovClustering<PointT, PointNT, PointLT>>;
  using ConstPtr =
      std::shared_ptr<const MarkovClustering<PointT, PointNT, PointLT>>;

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

  using VisualizationCallback = std::function<void(
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr,
      std::shared_ptr<std::vector<pcl::Vertices>>, CorrespondencesPtr)>;

    protected:
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  // using PCLBase<PointT>::initCompute;
  // using PCLBase<PointT>::deinitCompute;

  using Vertex = struct Vertex {
    float x, y, z, r; // x, y, z coordinates and radius
  };

  using Normal = struct Normal {
    float x, y, z; // x, y, z components of the normal vector
  };

    public:
  /** \brief Constructor for OrganizedMultiPlaneSegmentation. */
  MarkovClustering() = default;

  /** \brief Destructor for OrganizedMultiPlaneSegmentation. */
  ~MarkovClustering() override = default;

  /** \brief Provide a pointer to the input normals.
   * \param[in] normals the input normal cloud
   */
  inline void setInputNormals(const PointCloudNConstPtr &normals) {
    normals_ = normals;
  }

  /** \brief Get the input normals. */
  inline PointCloudNConstPtr getInputNormals() const { return (normals_); }

  inline void setInflationFactor(double inflation) { inflation_ = inflation; }

  inline double getInflationFactor() const { return inflation_; }

  inline void setExpansionFactor(int expansion) { expansion_ = expansion; }

  inline int getExpansionFactor() const { return expansion_; }

  inline void setPruningThreshold(double threshold) {
    pruning_threshold_ = threshold;
  }

  inline double getPruningThreshold() const { return pruning_threshold_; }

  inline void setConvergenceThreshold(double threshold) {
    convergence_threshold_ = threshold;
  }

  inline double getConvergenceThreshold() const {
    return convergence_threshold_;
  }

  inline void setMaxIterations(int max_iter) { max_iter_ = max_iter; }

  inline int getMaxIterations() const { return max_iter_; }

  inline void setRadius(float radius) { radius_ = radius; }

  inline float getRadius() const { return radius_; }

  inline void setGridSize(float grid_size) { radius_ = grid_size * M_SQRT2; }

  inline float getGridSize() const { return radius_ / M_SQRT2; }

  inline size_t getNumClusters() const { return numClusters_; }

  inline void registerVisualizationCallback(VisualizationCallback callback) {
    visualization_callback_ = callback;
  }

  /** \brief Segmentation of all planes in a point cloud given by
   * setInputCloud(), setIndices()
   * \param[out] labels a point cloud for the connected component labels of each
   * point in the input cloud.
   */
  void cluster(PointCloudL &labels) {

    spdlog::trace("Initializing segmentation class");
    if (!initCompute()) {
      deinitCompute();
      return;
    }

    createScene();

    std::vector<std::tuple<IT, IT, NT>> triplets;

    rayTracing(triplets);

    if (triplets.empty())
      spdlog::warn("No triplets found during ray tracing. Exiting.");

    if (visualization_callback_) {
      spdlog::trace("Creating visualization context");
      using CloudV = pcl::PointCloud<pcl::PointXYZ>;
      using CloudVPtr = CloudV::Ptr;
      using Vertices = std::vector<pcl::Vertices>;
      using VecticesPtr = std::shared_ptr<Vertices>;

      CloudVPtr points(new CloudV);
      VecticesPtr vertices(new Vertices);
      create_visualization_context(points, vertices);
      CorrespondencesPtr correspondences(new Correspondences);

      correspondences->resize(triplets.size());
      for (size_t i = 0; i < triplets.size(); ++i) {
        const auto &[s, t, v] = triplets[i];
        (*correspondences)[i].index_query = static_cast<int>(s * (N_ + 1));
        (*correspondences)[i].index_match = static_cast<int>(t * (N_ + 1));
      }

      visualization_callback_(points, vertices, correspondences);
    }

#if USE_CLI
    // INFO: Define output paths
    std::filesystem::path abc_path_in =
        std::filesystem::current_path() / "matrix.abc";
    std::filesystem::path abc_path_out =
        std::filesystem::current_path() /
        fmt::format("matrix.abc.out.I{}", inflation_);

    // INFO: Save the triples to disk in abc format
    std::ofstream abc_file(abc_path_in);
    // Set precision to 6 decimal places
    abc_file << std::fixed << std::setprecision(1);
    for (const auto &[i, j, v] : triplets) {
      abc_file << i << " " << j << " " << v << "\n";
    }
    abc_file.close();

    // Call mcl on the saved file
    try {

      // Construct the command
      auto cmd = fmt::format(
          "mcl {} --abc -I {} -te {} -o {}", abc_path_in.string(), inflation_,
          std::thread::hardware_concurrency(), abc_path_out.string());

      // Execute the command and capture output
      std::array<char, 128> buffer;
      std::string result;
      std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.data(), "r"),
                                                    pclose);
      if (!pipe) {
        throw std::runtime_error("popen() failed!");
      }
      while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        std::cout << buffer.data();

      // Read the output file and assign labels
      labels.resize(input_->size());
      std::ifstream file(abc_path_out);
      if (file.is_open()) {
        std::string line;
        size_t cls_i = 0, mem_j = 0;
        while (std::getline(file, line)) {
          std::istringstream iss(line);
          // int mem_j;
          while (iss >> mem_j)
            labels.points[indices_->at(mem_j)].label =
                static_cast<unsigned int>(cls_i);
          cls_i++;
        }
        file.close();
        numClusters_ = cls_i;
      } else
        spdlog::error("Unable to open output file from mcl: {}", abc_path_out);
    } catch (const std::exception &e) {
      spdlog::error("Exception during MCL execution: {}", e.what());
    }
#else
    GrB_Matrix M = NULL;
    LAGRAPH_TRY(
        GrB_Matrix_new(&M, GrB_FP32, indices_->size(), indices_->size()));
    for (const auto &[i, j, v] : triplets) {
      LAGRAPH_TRY(GrB_Matrix_setElement_FP32(M, 1.0, i, j));
    }
    triplets.clear();

    // // INFO: Save the matric to disk for debugging
    // std::FILE *fout = fopen("./mcl.mtx", "w");
    // if (LAGraph_MMWrite(M, fout, NULL, msg) != GrB_SUCCESS)
    //   spdlog::error("LAGraph_MMWrite failed! {}", std::string(msg));
    // fclose(fout);

    // LAGRAPH_TRY(GrB_Matrix_free(&M));
    // FILE *f = fopen("/home/mephisto/mcl.mtx", "r");
    // LAGRAPH_TRY(LAGraph_MMRead(&M, f, msg));

    GRB_TRY(GrB_Matrix_set_INT32(M, 32, GxB_ROWINDEX_INTEGER_HINT));
    GRB_TRY(GrB_Matrix_set_INT32(M, 32, GxB_COLINDEX_INTEGER_HINT));
    GRB_TRY(GrB_Matrix_set_INT32(M, 32, GxB_OFFSET_INTEGER_HINT));

    // INFO: Symmetrize the matrix
    LAGRAPH_TRY(GrB_Matrix_eWiseAdd_BinaryOp(M, NULL, NULL, GrB_PLUS_FP32, M, M,
                                             GrB_DESC_T1));

    show_matrix(M, "Stochastic Matrix - Before");

    spdlog::trace("Running Markov Clustering");
    spdlog::stopwatch sw;
    LAGRAPH_TRY(LAGraph_New(&G, &M, LAGraph_ADJACENCY_UNDIRECTED, msg));
    G->is_symmetric_structure = LAGraph_TRUE;

    // FIXME: Add optional visualization of stochastic matrix for debugging
    // category=Geometry estimate=1h
    // Currently commented out LAGraph_Graph_Print call for debugging matrix state.
    // Should make this controllable via debug flag or verbosity level rather than
    // requiring code changes. Useful for validating graph structure during development.
    // LAGRAPH_TRY(LAGraph_Graph_Print(G, LAGraph_SHORT_VERBOSE, stdout, msg));

    spdlog::debug("LAGraph is symmetric: {}",
                  G->is_symmetric_structure == LAGraph_TRUE ? "true" : "false");
    spdlog::debug("LAGraph kind: {}", G->kind == LAGraph_ADJACENCY_UNDIRECTED
                                          ? "undirected"
                                          : "directed");

    if (LAGr_MarkovClustering(&c, expansion_, inflation_, pruning_threshold_,
                              convergence_threshold_, max_iter_, G,
                              msg) != GrB_SUCCESS)
      spdlog::error("LAGr_MarkovClustering failed! {}", std::string(msg));
    spdlog::info("Markov Clustering completed in {:.3f} seconds", sw);

    extractClusters(labels);
#endif
    deinitCompute();
  }

  void rayTracing(std::vector<std::tuple<IT, IT, NT>> &triplets) {

    spdlog::trace("Running ray tracing");
    spdlog::stopwatch sw;

    const size_t N = indices_->size();
    auto logger = spdmon::LoggerProgress("Ray tracing", N * (N - 1) / 2);

#pragma omp parallel for reduction(mergeTriplets : triplets)                   \
    schedule(dynamic, 4)
    for (size_t i = 0; i < N; i++) {
      for (size_t j = i + 1; j < N; j++) {
        const size_t idx_s = indices_->at(i);
        const size_t idx_t = indices_->at(j);

        Eigen::Vector3f point_s = input_->points[idx_s].getVector3fMap();
        Eigen::Vector3f point_t = input_->points[idx_t].getVector3fMap();

        Eigen::Vector3f normal_s =
            normals_->points[idx_s].getNormalVector3fMap();
        Eigen::Vector3f normal_t =
            normals_->points[idx_t].getNormalVector3fMap();

        point_s = point_s + (offset_ * normal_s);
        point_t = point_t + (offset_ * normal_t);

        RTCRay ray;
        ray.org_x = point_s.x();
        ray.org_y = point_s.y();
        ray.org_z = point_s.z();

        Eigen::Vector3f dir = point_t - point_s;

        ray.tnear = epsilon_;
        const auto dir_norm = dir.norm();
        ray.tfar = dir_norm - epsilon_;

        dir.normalize();

        ray.dir_x = dir.x();
        ray.dir_y = dir.y();
        ray.dir_z = dir.z();

        ray.time = 0.0f; // motion blur time, not used here
        ray.mask = -1;   // mask for ray types, not used here
        ray.id = 0;      // ray ID, not used here
        ray.flags = 0;   // ray flags, not used here

        ++logger;
        rtcOccluded1(scene_, &ray);

        // When no intersection is found, the ray data is not updated. Incase
        // a hit was found, the tfar component of the ray is set to -inf.
        if (ray.tfar == -INFINITY) // A.k.a An intersection was found
          continue;

        triplets.emplace_back(i, j, 1.0);
      }
    }
    spdlog::debug("Ray tracing completed in {} seconds", sw);
  }

    protected:
  /** \brief Initialize the segmentation process.
   * \return true if initialization was successful, false otherwise
   */
  bool initCompute() {
    spdlog::trace("Initializing MarkovClustering");

    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    spdlog::trace("Creating device and scene for ray tracing");
    device_ = rtcNewDevice("verbose=0"); // 0-3
    scene_ = rtcNewScene(device_);
    rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
    assert(device_ != nullptr && "Error creating Embree device");
    assert(scene_ != nullptr && "Error creating Embree scene");
    rtcSetDeviceErrorFunction(device_, rtcCheckError, nullptr);

    spdlog::trace("Checking if input cloud is set");
    if (!PCLBase<PointT>::initCompute()) {
      PCL_ERROR("[pcl::%s::initCompute] Init failed.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

    spdlog::trace("Checking if input normals are set");
    if (!normals_) {
      PCL_ERROR("[pcl::%s::segment] Must specify normals.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

    spdlog::trace("Checking if input normals size matches input cloud size");
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

    spdlog::trace("Checking if input normals are normalized");
    for (const auto &point : normals_->points) {
      if ((point.getNormalVector3fMap().norm() - 1.0f) > epsilon_) {
        PCL_WARN("[pcl::%s::initCompute] Input normals are not normalized.\n",
                 getClassName().c_str());
        return (false);
      }
    }

    spdlog::trace("Checking if input normals contain NaN values");
    for (const auto &point : normals_->points) {
      if (std::isnan(point.normal_x) || std::isnan(point.normal_y) ||
          std::isnan(point.normal_z)) {
        PCL_WARN("[pcl::%s::initCompute] Input normals contain NaN values.\n",
                 getClassName().c_str());
        return (false);
      }
    }

    LAGraph_Init(msg);
    // GrB_Vector_new(&c, GrB_INT32, indices_->size());

    spdlog::debug("Initilized with with:");
    spdlog::debug("inflation: {:.3}", inflation_);
    spdlog::debug("expansion: {}", expansion_);
    spdlog::debug("pruning: {:.6f}", pruning_threshold_);
    spdlog::debug("convergence: {:.9f}", convergence_threshold_);
    spdlog::debug("max_iter: {}", max_iter_);
    spdlog::debug("radius: {:.3f}", radius_);

    return (true);
  }

  void deinitCompute() {
    spdlog::trace("Deinitializing MarkovClustering");
    pcl::PCLBase<PointT>::deinitCompute();
    spdlog::trace("Releasing scene and device");
    rtcReleaseScene(scene_);
    rtcReleaseDevice(device_);

    _mm_setcsr(old_mxcsr); // restore old flags

    GrB_Vector_free(&c);
    LAGraph_Delete(&G, msg);
    LAGraph_Finalize(msg);
  }

  /** \brief Create the scene geometry for the ray tracing.
   * This function creates an oriented disc point geometry in the scene.
   */
  void createScene() {
    spdlog::trace("Creating scene geometry for ray tracing");

    geometry_ = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    Vertex *vb = (Vertex *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vertex),
        indices_->size());

    Normal *nb = (Normal *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, sizeof(Normal),
        indices_->size());

    for (size_t i = 0; i < indices_->size(); ++i) {
      const auto id = indices_->at(i);
      const auto &p = input_->points[id];
      const auto &n = normals_->points[id];

      vb[i].x = static_cast<float>(p.x);
      vb[i].y = static_cast<float>(p.y);
      vb[i].z = static_cast<float>(p.z);
      vb[i].r = static_cast<float>(radius_) * 1.05f;

      nb[i].x = static_cast<float>(n.normal_x);
      nb[i].y = static_cast<float>(n.normal_y);
      nb[i].z = static_cast<float>(n.normal_z);
    }

    rtcSetGeometryMask(geometry_, 0xFFFFFFFF);

    rtcCommitGeometry(geometry_);
    rtcAttachGeometry(scene_, geometry_);
    rtcReleaseGeometry(geometry_);

    spdlog::trace("Committing scene");
    rtcCommitScene(scene_);
  }

  static void rtcCheckError(void *, RTCError err, const char *str) {
    spdlog::error("Embree error {}:", str);
    switch (err) {
    case RTC_ERROR_NONE:
      break;
    case RTC_ERROR_UNKNOWN:
      throw std::runtime_error("Embree: An unknown error has occurred.");
      break;
    case RTC_ERROR_INVALID_ARGUMENT:
      throw std::runtime_error("Embree: An invalid argument was specified.");
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
  }

  void extractClusters(PointCloudL &labels) {
    GrB_Index n;
    LAGRAPH_TRY(GrB_Matrix_nrows(&n, G->A));

    // FIXME: Add optional visualization of cluster assignment vector
    // category=Geometry estimate=1h
    // Commented LAGraph_Vector_Print for debugging cluster assignments.
    // Should integrate with spdlog debug levels instead of requiring code edits.
    // Helpful for verifying cluster convergence and label distribution.
    // LAGRAPH_TRY(LAGraph_Vector_Print(c, LAGraph_SHORT_VERBOSE, stdout, msg));

    // INFO: Extract labels from converged matrix
    assert(n == indices_->size());
    labels.resize(input_->size());
    for (GrB_Index i = 0; i < n; ++i) {
      int label = -1;
      LAGRAPH_TRY(GrB_Vector_extractElement_INT32(&label, c, i));
      labels.points[indices_->at(i)].label = static_cast<unsigned int>(label);
    }

    double cov, perf, mod;
    LAGRAPH_TRY(LAGr_PartitionQuality(&cov, &perf, c, G, msg));
    LAGRAPH_TRY(LAGr_Modularity(&mod, (double)1, c, G, msg));
    spdlog::info("Partition quality: coverage: {:.3f}, performance: {:.3f}, "
                 "modularity: {:.3f}",
                 cov, perf, mod);

    GrB_Index nclusters;
    LAGRAPH_TRY(GrB_Vector_nvals(&nclusters, c));

    GrB_Vector vpc, vpc_sorted = NULL;
    GrB_Matrix C = NULL;
    GrB_Scalar TRUE_BOOL = NULL;
    GrB_Index *cI, *cX = NULL;

    LAGRAPH_TRY(GrB_Matrix_new(&C, GrB_BOOL, n, n));
    LAGRAPH_TRY(GrB_Vector_new(&vpc, GrB_INT64, n));
    LAGRAPH_TRY(GrB_Vector_new(&vpc_sorted, GrB_INT64, n));
    LAGRAPH_TRY(GrB_Scalar_new(&TRUE_BOOL, GrB_BOOL));

    LAGRAPH_TRY(
        LAGraph_Malloc((void **)&cI, nclusters, sizeof(GrB_Index), msg));
    LAGRAPH_TRY(
        LAGraph_Malloc((void **)&cX, nclusters, sizeof(GrB_Index), msg));
    LAGRAPH_TRY(GrB_Scalar_setElement_BOOL(TRUE_BOOL, (bool)1));

    LAGRAPH_TRY(
        GrB_Vector_extractTuples_INT64(cI, (int64_t *)cX, &nclusters, c));
    LAGRAPH_TRY(GxB_Matrix_build_Scalar(C, cX, cI, TRUE_BOOL, nclusters));

    LAGRAPH_TRY(GrB_Matrix_reduce_Monoid(vpc, NULL, NULL, GrB_PLUS_MONOID_INT64,
                                         C, NULL));
    LAGRAPH_TRY(GxB_Vector_sort(vpc_sorted, NULL, GrB_GT_FP32, vpc, NULL));

    // FIXME: Add visualization of sorted votes-per-cluster vector
    // category=Geometry estimate=1h
    // Commented debug print for sorted cluster sizes. Useful for understanding
    // cluster size distribution and detecting degenerate cases (too many/few clusters).
    // Should be controlled by debug flags rather than code modification.
    // LAGRAPH_TRY(
    //     LAGraph_Vector_Print(vpc_sorted, LAGraph_SHORT_VERBOSE, stdout,
    //     msg));

    LAGRAPH_TRY(GrB_Vector_nvals(&numClusters_, vpc_sorted));
  }

  void create_visualization_context(
      pcl::PointCloud<pcl::PointXYZ>::Ptr points,
      std::shared_ptr<std::vector<pcl::Vertices>> vertices) {

    Vertex *vb = (Vertex *)rtcGetGeometryBufferData(geometry_,
                                                    RTC_BUFFER_TYPE_VERTEX, 0);
    Normal *nb = (Normal *)rtcGetGeometryBufferData(geometry_,
                                                    RTC_BUFFER_TYPE_NORMAL, 0);
    assert(vb != nullptr && nb != nullptr);

    points->resize(indices_->size() * (N_ + 1)); // +1 for center point
    vertices->resize(indices_->size());

    for (size_t i = 0; i < indices_->size(); ++i) {
      pcl::Vertices *circle = &(*vertices)[i];
      circle->vertices.resize(N_);

      Eigen::Vector3d center(vb[i].x, vb[i].y, vb[i].z);
      Eigen::Vector3d dir(nb[i].x, nb[i].y, nb[i].z);

      points->points[i * (N_ + 1)] =
          pcl::PointXYZ(center.x(), center.y(), center.z());

      dir = dir.normalized();

      // pick an arbitrary vector not parallel to n
      Eigen::Vector3d arbitrary(1.0, 0.0, 0.0);
      if (std::fabs(dir.dot(arbitrary)) > 0.9) { // too close to parallel
        arbitrary = Eigen::Vector3d(0.0, 1.0, 0.0);
      }

      Eigen::Vector3d u, v;
      u = dir.cross(arbitrary).normalized(); // first basis vector
      v = dir.cross(u).normalized();         // second basis vector

      for (unsigned int j = 0; j < N_; ++j) {

        double theta = 2.0 * M_PI * j / N_;
        Eigen::Vector3d local = radius_ * (cos(theta) * u + sin(theta) * v);
        Eigen::Vector3d global = center + local;

        const size_t idx = i * (N_ + 1) + (j + 1);
        points->points[idx] = pcl::PointXYZ(global.x(), global.y(), global.z());
        circle->vertices[j] = static_cast<uint32_t>(idx);
      }
    };
  }

  void show_matrix(GrB_Matrix m, const std::string &name = "Matrix",
                   unsigned int w = 1000, unsigned int h = 1000) {

    if (m == NULL) {
      spdlog::warn("Matrix {} is NULL", name);
      return;
    }

    GrB_Index nrows = 0, ncols = 0, nvals = 0;
    LAGRAPH_TRY(GrB_Matrix_nrows(&nrows, m));
    LAGRAPH_TRY(GrB_Matrix_ncols(&ncols, m));
    LAGRAPH_TRY(GrB_Matrix_nvals(&nvals, m));

    unsigned int w_, h_;
    w_ = w;
    h_ = h;

    if (nrows < h)
      h = static_cast<unsigned int>(nrows);

    if (ncols < w)
      w = static_cast<unsigned int>(ncols);

    cv::Mat mat(w, h, CV_32FC1, cv::Scalar(0));
    std::vector<GrB_Index> rows(nvals);
    std::vector<GrB_Index> cols(nvals);
    std::vector<NT> vals(nvals);

    rows.resize(nvals);
    cols.resize(nvals);
    vals.resize(nvals);

    LAGRAPH_TRY(GrB_Matrix_extractTuples_FP32(rows.data(), cols.data(),
                                              vals.data(), &nvals, m));

    for (size_t i = 0; i < nvals; ++i) {
      int x = static_cast<int>(cols[i] * (w - 1) / nrows);
      int y = static_cast<int>(rows[i] * (h - 1) / nrows);
      x = std::min(x, static_cast<int>(w - 1));
      y = std::min(y, static_cast<int>(h - 1));
      mat.at<float>(y, x) += static_cast<float>(vals[i]);
    }

    // Scale image to original size
    if (w != w_ || h != h_) {
      spdlog::debug("Resizing image from {}x{} to {}x{}", w, h, w_, h_);
      cv::resize(mat, mat, cv::Size(w_, h_), 0, 0, cv::INTER_NEAREST);
    }

    cv::normalize(mat, mat, 0, 1, cv::NORM_MINMAX);
    cv::imshow(name, mat);

    cv::Mat mat8u;
    mat.convertTo(mat8u, CV_8UC1, 255.0);
    cv::imwrite("./" + name + ".png", mat8u);

#if 0
    cv::waitKey(10);
#else
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
  }

  /** \brief Get a string representation of the name of this class. */
  inline const std::string getClassName() const { return ("MarkovClustering"); }

    protected:
  /** \brief A pointer to the input normals */
  PointCloudNConstPtr normals_{nullptr};

  long unsigned int numClusters_ = 0;

  float offset_{0.10f};
  float radius_{0.05f};

  RTCDevice device_{};
  RTCScene scene_{};
  RTCGeometry geometry_{};

  // TODO: Expose MCL algorithm parameters via public setter methods
  // category=Geometry estimate=2h
  // Currently expansion, inflation, and pruning_threshold are hardcoded private members.
  // Should add public setter/getter methods to allow tuning MCL behavior:
  // - setExpansion(int): Controls matrix expansion iterations
  // - setInflation(double): Controls cluster granularity
  // - setPruningThreshold(double): Controls sparsity pruning
  // Enables parameter tuning without recompilation for upstream PCL contribution
  int expansion_{2};
  double inflation_{2.0};
  double pruning_threshold_{0.0001};
  double convergence_threshold_{1e-8};
  int max_iter_{100};

  char msg[LAGRAPH_MSG_LEN];
  LAGraph_Graph G = NULL;
  GrB_Vector c = NULL;

  VisualizationCallback visualization_callback_{nullptr};
  static constexpr unsigned int N_ =
      6; // Number of vertices to represent a disc

  constexpr static double epsilon_ = 1e-4;

  unsigned int old_mxcsr = _mm_getcsr(); // save current flagsq

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

#pragma once
#include <mpi.h>

#include <cmath>

#include <pcl/PointIndices.h>
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
#include "core/spdmon.hh"
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>

using IT = int64_t; // Index type
using NT = double;  // Numeric type

// Define OpenMP reduction for std::vector<std::tuple<int64_t, int64_t,
// double>>
#pragma omp declare reduction(                                                 \
        mergeTriplets : std::vector<std::tuple<IT, IT, NT>> : omp_out.insert(  \
                omp_out.end(), omp_in.begin(), omp_in.end()))                  \
    initializer(omp_priv = std::vector<std::tuple<IT, IT, NT>>())

// Only during CMAKE_BUILD_TYPE=Debug, CHECK will check the return value
// #ifdef NDEBUG
#if 1
#define CHECK(info)                                                            \
  do {                                                                         \
    GrB_Info _info = (info);                                                   \
    if (_info != GrB_SUCCESS) {                                                \
      fprintf(stderr, "GraphBLAS error %d at %s:%d\n", (int)_info, __FILE__,   \
              __LINE__);                                                       \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)
// Else do nothing
#else
#define CHECK(info) (info)
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

    spdlog::trace("Running ray tracing");
    spdlog::stopwatch sw;
    std::vector<std::tuple<IT, IT, NT>> triplets;

    // INFO: Ray tracing
    {
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
          memset(&ray, 0, sizeof(ray));
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

          // FIXME: This should not be necessary as memset should already have
          // set everything to zero
          ray.time = 0.0f;       // motion blur time, not used here
          ray.mask = 0xFFFFFFFF; // mask for ray types, not used here
          ray.id = 0;            // ray ID, not used here
          ray.flags = 0;         // ray flags, not used here

          ++logger;
          rtcOccluded1(scene_, &ray);

          // When no intersection is found, the ray data is not updated. Incase
          // a hit was found, the tfar component of the ray is set to -inf.
          if (ray.tfar != -INFINITY) // A.k.a An intersection was found
            continue;

          triplets.emplace_back(i, j, 1.0);
        }
      }
    }

    spdlog::debug("Ray tracing completed in {} seconds", sw);

    if (triplets.empty()) {
      spdlog::warn("No triplets found during ray tracing. Exiting.");
    }

    GrB_Matrix M = NULL;
    GrB_Matrix_new(&M, GrB_BOOL, indices_->size(), indices_->size());
    for (const auto &[i, j, v] : triplets) {
      GrB_Matrix_setElement_BOOL(M, true, i, j);
    }
    triplets.clear();

    // INFO: Save the matric to disk for debugging
    std::FILE *fout = fopen("./mcl.mtx", "w");
    if (LAGraph_MMWrite(M, fout, NULL, msg) != GrB_SUCCESS)
      spdlog::error("LAGraph_MMWrite failed! {}", std::string(msg));
    fclose(fout);

    // INFO: Symmetrize the matrix
    GrB_Matrix_eWiseAdd_BinaryOp(M, NULL, NULL, GrB_PLUS_FP64, M, M,
                                 GrB_DESC_T1);

    // GrB_Matrix_free(&M);
    // FILE *f = fopen("/home/mephisto/mcl.mtx", "r");
    // LAGraph_MMRead(&M, f, msg);
    // show_matrix(M, "Stochastic Matrix - Before");

    spdlog::trace("Running Markov Clustering");
    sw.reset();
    LAGraph_New(&G, &M, LAGraph_ADJACENCY_UNDIRECTED, msg);
    G->is_symmetric_structure = LAGraph_TRUE;

    LAGraph_Graph_Print(G, LAGraph_SHORT_VERBOSE, stdout, msg);

    if (LAGr_MarkovClustering(&c, expansion_, inflation_, pruning_threshold_,
                              convergence_threshold_, max_iter_, G,
                              msg) != GrB_SUCCESS)
      spdlog::error("LAGr_MarkovClustering failed! {}", std::string(msg));
    spdlog::info("Markov Clustering completed in {:.3f} seconds", sw);

    // FIXME: Visual debug the stochastic matrix
    show_matrix(G->A, "Stochastic Matrix");

    LAGraph_Vector_Print(c, LAGraph_SHORT_VERBOSE, stdout, msg);

    double cov, perf, mod;
    LAGr_PartitionQuality(&cov, &perf, c, G, msg);
    spdlog::info("Partition quality: coverage: {:.6f}, performance: {:.6f}, "
                 "modularity: {:.6f}",
                 cov, perf, mod);

    LAGr_Modularity(&mod, (double)1, c, G, msg);
    spdlog::info("Modularity: {:.6f}", mod);

    GrB_Index nclusters;
    GrB_Vector_nvals(&nclusters, c);
    spdlog::info("Number of clusters (from nvals): {}", nclusters);

    GrB_Vector vpc, vpc_sorted = NULL;
    GrB_Matrix C = NULL;
    GrB_Scalar TRUE_BOOL = NULL;

    GrB_Index *cI, *cX = NULL;

    GrB_Matrix_new(&C, GrB_BOOL, indices_->size(), indices_->size());
    GrB_Vector_new(&vpc, GrB_INT64, indices_->size());
    GrB_Vector_new(&vpc_sorted, GrB_INT64, indices_->size());

    LAGraph_Malloc((void **)&cI, nclusters, sizeof(GrB_Index), msg);
    LAGraph_Malloc((void **)&cX, nclusters, sizeof(GrB_Index), msg);
    GrB_Vector_extractTuples_INT64(cI, (int64_t *)cX, &nclusters, c);
    GxB_Matrix_build_Scalar(C, cX, cI, TRUE_BOOL, nclusters);

    GrB_Matrix_reduce_Monoid(vpc, NULL, NULL, GrB_PLUS_MONOID_INT64, C, NULL);
    GxB_Vector_sort(vpc_sorted, NULL, GrB_GT_FP64, vpc, NULL);

    LAGraph_Vector_Print(vpc_sorted, LAGraph_SHORT_VERBOSE, stdout, msg);

    // // INFO: Extract labels from converged matrix
    // GrB_Index nvals = 0;
    // GrB_Vector_nvals(&nvals, c);
    // assert(nvals == indices_->size());
    // labels.resize(input_->size());
    // for (GrB_Index i = 0; i < nvals; ++i) {
    //   int label = -1;
    //   GrB_Vector_extractElement_INT32(&label, c, i);
    //   labels.points[indices_->at(i)].label = label;
    // }

    // // FIXME: Set numClusters_
    // int32_t max_label = 0;
    // GrB_Vector_reduce_INT32(&max_label, NULL, GrB_MAX_MONOID_INT32, c, NULL);
    // numClusters_ = static_cast<unsigned int>(max_label) + 1;
    // spdlog::debug("Number of clusters found: {}", numClusters_);

    deinitCompute();
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

    spdlog::debug("Initilized with inflation: {}, expansion: {}, pruning: "
                  "{:.6f}, convergence: {:.6f}, max_iter: {}, radius: {:.3f}",
                  inflation_, expansion_, pruning_threshold_,
                  convergence_threshold_, max_iter_, radius_);

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

    RTCGeometry geom =
        rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    Vertex *vb = (Vertex *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vertex),
        indices_->size());

    Normal *nb = (Normal *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, sizeof(Normal),
        indices_->size());

    for (size_t i = 0; i < indices_->size(); ++i) {
      const auto id = indices_->at(i);
      const auto &p = input_->points[id];
      const auto &n = normals_->points[id];

      vb[i].x = static_cast<float>(p.x);
      vb[i].y = static_cast<float>(p.y);
      vb[i].z = static_cast<float>(p.z);
      vb[i].r = static_cast<float>(radius_) + radius_ * 0.05f;

      nb[i].x = static_cast<float>(n.normal_x);
      nb[i].y = static_cast<float>(n.normal_y);
      nb[i].z = static_cast<float>(n.normal_z);
    }

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene_, geom);
    rtcReleaseGeometry(geom);

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

  static void show_matrix(GrB_Matrix m, const std::string &name = "Matrix",
                          unsigned int w = 1000, unsigned int h = 1000) {

    if (m == NULL) {
      spdlog::warn("Matrix {} is NULL", name);
      return;
    }

    GrB_Index nrows = 0, ncols = 0, nvals = 0;
    CHECK(GrB_Matrix_nrows(&nrows, m));
    CHECK(GrB_Matrix_ncols(&ncols, m));
    CHECK(GrB_Matrix_nvals(&nvals, m));

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
    std::vector<double> vals(nvals);

    rows.resize(nvals);
    cols.resize(nvals);
    vals.resize(nvals);

    CHECK(GrB_Matrix_extractTuples_FP64(rows.data(), cols.data(), vals.data(),
                                        &nvals, m));

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

  unsigned int numClusters_ = 0;

  float offset_{0.10f};
  float radius_{0.05f};

  RTCDevice device_{};
  RTCScene scene_{};

  // TODO: Make these parameters settable
  int expansion_{2};
  double inflation_{2.0};
  double pruning_threshold_{0.0001};
  double convergence_threshold_{1e-8};
  int max_iter_{100};

  char msg[LAGRAPH_MSG_LEN];
  LAGraph_Graph G = NULL;
  GrB_Vector c = NULL;

  constexpr static double epsilon_ = 0.00001;

  unsigned int old_mxcsr = _mm_getcsr(); // save current flags

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

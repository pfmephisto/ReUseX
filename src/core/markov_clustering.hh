#pragma once
#include <cmath>

#include <pcl/PointIndices.h>
#include <pcl/common/pca.h>
#include <pcl/common/utils.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <Eigen/SparseCore>

#include <embree4/rtcore.h>

#include "core/spdmon.hh"
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "pmmintrin.h"
#include "xmmintrin.h" // for SSE intrinsics

// Define OpenMP reduction for std::vector<Eigen::Triplet<double>>
#pragma omp declare reduction(                                                 \
        mergeTriplets : std::vector<Eigen::Triplet<double>> : omp_out.insert(  \
                omp_out.end(), omp_in.begin(), omp_in.end()))                  \
    initializer(omp_priv = std::vector<Eigen::Triplet<double>>())

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
  using SpMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;
  using InflationFunc = std::function<double(double)>;

  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  // using PCLBase<PointT>::initCompute;
  // using PCLBase<PointT>::deinitCompute;
  using Triplet = Eigen::Triplet<double>;

  using Vertex = struct Vertex {
    float x, y, z, r; // x, y, z coordinates and radius
  };

  using Normal = struct Normal {
    float x, y, z; // x, y, z components of the normal vector
  };

  // struct Vertex {
  //   float x, y, z, r;
  // };

  // struct Normal {
  //   float x, y, z;
  // };

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

  inline void setRadius(float radius) { radius_ = radius; }

  inline float getRadius() const { return radius_; }

  inline void setGridSize(float grid_size) {
    radius_ = grid_size / 2 * M_SQRT2;
  }

  inline float getGridSize() const { return radius_ / M_SQRT2 * 2; }

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
    std::vector<Triplet> triplets;

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

          ray.time = 0.0f;       // motion blur time, not used here
          ray.mask = 0xFFFFFFFF; // mask for ray types, not used here
          ray.id = 0;            // ray ID, not used here
          ray.flags = 0;         // ray flags, not used here

          ++logger;
#if 1
          rtcOccluded1(scene_, &ray);

          // When no intersection is found, the ray data is not updated. Incase
          // a hit was found, the tfar component of the ray is set to -inf.
          if (ray.tfar != -INFINITY) // A.k.a An intersection was found
            continue;
#else
          // Use rtcIntersect1 instead of rtcOccluded1
          RTCRayHit rayhit;
          memset(&rayhit, 0, sizeof(rayhit));
          rayhit.ray = ray;
          rtcIntersect1(scene_, &rayhit);

          if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            continue;
#endif

          // spdlog::trace("Ray from {} to {} did not hit anything", idx_s,
          // idx_t);

          triplets.emplace_back(idx_s, idx_t, 1.0);
          // triplets.emplace_back(idx_s, idx_s, 1.0); // Add self-loop
        }
      }
    }
    spdlog::debug("Ray tracing completed in {} seconds", sw);

    if (triplets.empty()) {
      spdlog::warn("No triplets found during ray tracing. Exiting.");
    }

    // Build sparse matrix once at the end
    spdlog::debug("Number of triplets: {}", triplets.size());
    matrix_.setFromTriplets(triplets.begin(), triplets.end());
    triplets.clear();

    // Add self-loop only if there are entries in the matrix
    // spdlog::trace("Adding self-loops to the matrix");
    // matrix_.setIdentity();
    // std::vector<char> visited(matrix_.rows(), 0);
    // std::vector<Triplet> selfLoops;
    // for (int k = 0; k < matrix_.outerSize(); ++k)
    //   for (SpMatrix::InnerIterator it(matrix_, k); it; ++it) {
    //     const auto r = it.row();
    //     if (visited[r])
    //       continue;
    //     selfLoops.emplace_back(r, r, 1.0);
    //     visited[r] = 1;
    //   }
    // matrix_.insertFromTriplets(triplets.begin(), triplets.end());
    // selfLoops.clear();

    spdlog::trace("Converting matrix to upper triangular form");
    matrix_ = matrix_.selfadjointView<Eigen::Upper>();

    runMCL();

    // INFO: Set the labels
    spdlog::trace("Setting labels for the points");
    labels.points.resize(input_->size());
    for (auto row = 0; row < matrix_.rows(); ++row) {
      int clusterId = -1;

      for (SpMatrix::InnerIterator it(matrix_, row); it; ++it) {
        if (it.row() == it.col() && it.value() > threshold_) {
          clusterId = numClusters_++;
        }
        if (clusterId != -1 && it.value() > threshold_) {
          labels.points[it.col()].label = clusterId;
        }
      }
    }
    spdlog::debug("Number of clusters found: {}", numClusters_);

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
    device_ = rtcNewDevice("verbose=3"); // 0-3
    scene_ = rtcNewScene(device_);
    rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
    spdlog::debug("Created device: {} and scene {}", fmt::ptr(device_),
                  fmt::ptr(scene_));
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

    matrix_.resize(input_->size(), input_->size());

    return (true);
  }

  void deinitCompute() {
    spdlog::trace("Deinitializing MarkovClustering");
    pcl::PCLBase<PointT>::deinitCompute();
    spdlog::trace("Releasing scene and device");
    rtcReleaseScene(scene_);
    rtcReleaseDevice(device_);
    spdlog::trace("Resize matrix_");
    matrix_.resize(0, 0);

    _mm_setcsr(old_mxcsr); // restore old flags
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
      vb[i].r = static_cast<float>(radius_);

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

  /** \brief Run the Markov Clustering algorithm on the matrix.
   * The matrix is normalized, expanded and inflated until convergence.
   */
  void runMCL() {
    spdlog::trace("Running Markov Clustering algorithm");
    spdlog::stopwatch sw;
    // INFO: Run the Markov Clustering algorithm

    normalize();
    for (size_t i = 0; i < max_loop_ && !stop(); ++i) {
      spdlog::debug("MCL Iteration {} of {}: Difference {}", i, max_loop_,
                    mcl_diff_);
      expand();
      inflate();
      normalize();
    }
    spdlog::debug("Markov Clustering completed in {} seconds", sw);
  }

  /** \brief Check if the matrix has converged.
   * The matrix is considered converged if the maximum and minimum
   * difference between the matrix and its square is zero.
   * \return true if the matrix has converged, false otherwise
   */
  inline bool stop() {
    mcl_diff_ = (matrix_ * matrix_ - matrix_).norm();
    return (mcl_diff_ < epsilon_);
  }

  void normalize() {
    Eigen::VectorXd colSums = Eigen::VectorXd::Zero(matrix_.cols());

    // accumulate sums
    for (int k = 0; k < matrix_.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix_, k); it;
           ++it) {
        colSums(it.col()) += it.value();
      }
    }

    Eigen::VectorXd invColSums = colSums.cwiseInverse();

    // scale in place
    for (int k = 0; k < matrix_.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix_, k); it;
           ++it) {
        it.valueRef() *= invColSums(it.col());
      }
    }
  }

  inline void expand() { matrix_ = matrix_ * matrix_; }

  inline void inflate() { matrix_ = matrix_.unaryExpr(inflation_func_); }

  /** \brief Get a string representation of the name of this class. */
  inline const std::string getClassName() const { return ("MarkovClustering"); }

    protected:
  /** \brief A pointer to the input normals */
  PointCloudNConstPtr normals_{nullptr};

  unsigned int numClusters_ = 0;

  /** \brief Inflation parameter for the clustering algorithm */
  double inflation_{2.0};

  float offset_{0.10f};
  float radius_{0.05f};

  RTCDevice device_{};

  RTCScene scene_{};

  // std::vector<RTCRay> rays_{};
  // std::vector<size_t> ray_source_indices_{};

  SpMatrix matrix_{};

  const InflationFunc inflation_func_{
      [this](double x) -> double { return std::pow(x, inflation_); }};

  double mcl_diff_{0.0};

  constexpr static size_t max_loop_ = 1000;

  constexpr static double epsilon_ = 0.00001;
  constexpr static double threshold_ = 0.5;

  unsigned int old_mxcsr = _mm_getcsr(); // save current flags

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

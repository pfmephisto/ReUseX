#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/angles.h>
#include <pcl/common/geometry.h>
#include <pcl/common/pca.h>
#include <pcl/common/utils.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/plane_refinement_comparator.h>

#include <queue>
#include <tbb/parallel_sort.h>

#include "core/spdmon.hh"
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

namespace pcl {

// Forward declaration of PlanarRegionGrowingComparator class
template <typename PointT, typename PointNT>
class PlanarRegionGrowingComparator;

/** \brief A class for segmenting planar regions in a point cloud using region
 * growing.
 *
 * This class implements a region growing algorithm to segment planar regions
 * in a point cloud based on the normal orientation and distance to the plane.
 * It uses PCA for plane fitting and can handle unorganized point clouds.
 *
 * \author Your Name
 */

template <typename PointT, typename PointNT = pcl::Normal,
          typename PointLT = pcl::Label, class Tree = pcl::KdTreeFLANN<PointT>>
class PCL_EXPORTS PlanarRegionGrowing : public PCLBase<PointT> {
    public:
  using Ptr =
      std::shared_ptr<PlanarRegionGrowing<PointT, PointNT, PointLT, Tree>>;
  using ConstPtr = std::shared_ptr<
      const PlanarRegionGrowing<PointT, PointNT, PointLT, Tree>>;

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

  using KdTreePtr = typename Tree::Ptr;

  using SearchMethod = std::function<int(
      const PointCloud &cloud, std::size_t index, pcl::Indices &out_indecies,
      std::vector<float> &out_distances)>;

  using PCA = pcl::PCA<PointT>;
  using PCAPtr = typename std::shared_ptr<PCA>;

  using PlaneComparator = PlanarRegionGrowingComparator<PointT, PointNT>;
  using PlaneComparatorPtr = typename PlaneComparator::Ptr;
  using PlaneComparatorConstPtr = typename PlaneComparator::ConstPtr;

    protected:
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  // using PCLBase<PointT>::initCompute;
  using PCLBase<PointT>::deinitCompute;

    public:
  /** \brief Constructor for OrganizedMultiPlaneSegmentation. */
  PlanarRegionGrowing() = default;

  /** \brief Destructor for OrganizedMultiPlaneSegmentation. */
  ~PlanarRegionGrowing() override = default;

  /** \brief Provide a pointer to the input normals.
   * \param[in] normals the input normal cloud
   */
  inline void setInputNormals(const PointCloudNConstPtr &normals) {
    normals_ = normals;
  }

  /** \brief Get the input normals. */
  inline PointCloudNConstPtr getInputNormals() const { return (normals_); }

  /** \brief Set the minimum number of inliers required for a plane
   * \param[in] min_inliers the minimum number of inliers required per plane
   */
  inline void setMinInliers(unsigned min_inliers) {
    min_inliers_ = min_inliers;
  }

  /** \brief Get the minimum number of inliers required per plane. */
  inline unsigned getMinInliers() const { return (min_inliers_); }

  /** \brief Set the tolerance in radians for difference in normal direction
   * between neighboring points, to be considered part of the same plane.
   * \param[in] angular_threshold the tolerance in radians
   */
  inline void setAngularThreshold(double angular_threshold) {
    angular_threshold_ = angular_threshold;
  }

  /** \brief Get the angular threshold in radians for difference in normal
   * direction between neighboring points, to be considered part of the same
   * plane. */
  inline double getAngularThreshold() const { return (angular_threshold_); }

  /** \brief Set the tolerance in meters for difference in perpendicular
   * distance (d component of plane equation) to the plane between neighboring
   * points, to be considered part of the same plane.
   * \param[in] distance_threshold the tolerance in meters
   */
  inline void setDistanceThreshold(double distance_threshold) {
    distance_threshold_ = distance_threshold;
  }

  /** \brief Get the distance threshold in meters (d component of plane
   * equation) between neighboring points, to be considered part of the same
   * plane. */
  inline double getDistanceThreshold() const { return (distance_threshold_); }

  /** \brief Provide a pointer to the search object.
   * \param[in] tree a pointer to the spatial search object.
   */
  inline void setSearchMethod(const KdTreePtr &tree) { tree_ = tree; }

  /** \brief Get a pointer to the search method used. */
  inline KdTreePtr getSearchMethod() const { return (tree_); }

  /** \brief Get the internal search parameter. */
  inline double getSearchParameter() const { return (search_parameter_); }

  /** \brief Set the number of k nearest neighbors to use for the feature
   * estimation.
   * \param[in] k the number of k-nearest neighbors
   */
  inline void setKSearch(int k) { k_ = k; }

  /** \brief get the number of k nearest neighbors used for the feature
   * estimation. */
  inline int getKSearch() const { return (k_); }

  /** \brief Set the sphere radius that is to be used for determining the
   * nearest neighbors used for the feature estimation.
   * \param[in] radius the sphere radius used as the maximum distance to
   * consider a point a neighbor
   */
  inline void setRadiusSearch(double radius) { search_radius_ = radius; }

  /** \brief Get the sphere radius used for determining the neighbors. */
  inline double getRadiusSearch() const { return (search_radius_); }

  /** \brief Provide a pointer to the comparator to be used for segmentation.
   * \param[in] compare A pointer to the comparator to be used for
   segmentation.
   */
  inline void setComparator(const PlaneComparatorPtr &compare) {
    compare_ = compare;
  }

  /** \brief Get the model coefficients for each plane found in the input cloud.
   * \return a vector of model coefficients for each plane found in the input
   */
  inline std::vector<pcl::ModelCoefficients> getModelCoefficients() const {
    return model_coefficients_;
  }

  /** \brief Get the inlier indices for each detected plane.
   * \return a vector of indices for each detected plane
   */
  inline std::vector<pcl::Indices> getInlierIndices() const {
    return inlier_indices_;
  }

  /** \brief Get the centroids for each plane found in the input cloud.
   * \return a vector of centroids for each plane found in the input cloud
   */
  inline std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
  getCentroids() const {
    return centroids_;
  }

  /** \brief Segmentation of all planes in a point cloud given by
   * setInputCloud(), setIndices()
   * \param[out] model_coefficients a vector of model_coefficients for each
   * plane found in the input cloud
   * \param[out] inlier_indices a vector of inliers for each detected plane
   * \param[out] centroids a vector of centroids for each plane
   * \param[out] covariances a vector of covariance matrices for the inliers of
   * each plane
   * \param[out] labels a point cloud for the connected component labels of each
   * pixel
   * \param[out] label_indices a vector of PointIndices for each labeled
   * component
   */
  void segment(const typename pcl::PointCloud<PointLT>::Ptr &labels) {
    // std::vector<ModelCoefficients> &model_coefficients,
    // std::vector<PointIndices> &inlier_indices,
    // std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
    //     &centroids) {

    spdlog::trace("Initializing segmentation class");
    if (!initCompute()) {
      deinitCompute();
      return;
    }

    spdlog::trace("Setting up the labels point cloud");
    // Check if the labels point cloud is empty
    if (labels->size() != input_->size())
      labels->resize(input_->size());

    // Reset the labels to 0
    for (size_t i = 0; i < labels->size(); ++i)
      labels->points[i].label = 0;

    labels_ = labels;

    spdlog::trace("Sorting indices");
    // INFO: Check if using tbb will make it faster
    // tbb::parallel_sort
    std::sort(indices_->begin(), indices_->end(), [&](int i, int j) {
      return normals_->at(i).curvature < normals_->at(j).curvature;
    });

    // TODO: Move this to custom mehthod for redability
    spdlog::trace("Precompute neighbor indices");
    spdlog::stopwatch sw;
    point_neighbours_.clear();
    point_neighbours_.resize(indices_->size(), pcl::Indices());
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(indices_->size()); ++i) {
      pcl::Indices k_indices;
      std::vector<float> k_distances;
      search_method_(*input_, indices_->at(i), k_indices, k_distances);
      std::swap(k_indices, point_neighbours_[i]);
    }
    spdlog::trace("Precompute neighbor indices took {}s", sw);

    size_t seed_counter = 0;
    size_t seed = static_cast<size_t>(indices_->at(seed_counter));

    size_t segmented_pts_num = 0;
    size_t number_of_segments = 0;

    auto logger = spdmon::LoggerProgress("Region Growing", indices_->size());

    while (segmented_pts_num < indices_->size()) { // TODO: Add early stop
      size_t pts_in_segment = 0;
      pts_in_segment = growRegion(seed, number_of_segments);
      segmented_pts_num += pts_in_segment;
      num_pts_in_segment_.push_back(pts_in_segment);
      number_of_segments++; // Increment the segment number

      logger += pts_in_segment; // Update the progress bar

      // Find the next seed point
      for (size_t i_seed = seed_counter + 1; i_seed < indices_->size();
           ++i_seed) {
        size_t index = indices_->at(i_seed);
        if (labels_->points[index].label == 0) {
          seed = index;
          seed_counter = i_seed;
          break;
        }
      }
    }

    // INFO: Refine segments
    spdlog::trace("Refining segments");
#pragma omp parallel for
    for (size_t i = 0; i < inlier_indices_.size(); ++i) {
      if (inlier_indices_[i].size() < min_inliers_) {
        for (size_t j = 0; j < inlier_indices_[i].size(); ++j) {
          size_t index = inlier_indices_[i][j];
          labels_->points[index].label = 0; // Reset the label
        }
      }
    }

    deinitCompute();
  }

    protected:
  /** \brief Initialize the segmentation process.
   * \return true if initialization was successful, false otherwise
   */
  bool initCompute() {

    if (!PCLBase<PointT>::initCompute()) {
      PCL_ERROR("[pcl::%s::initCompute] Init failed.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

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

    // Check if a space search locator was given
    if (!tree_) {
      PCL_DEBUG("[pcl::%s::initCompute] No search tree given, creating a new "
                "one.\n",
                getClassName().c_str());
      tree_.reset(new Tree(false));
      tree_->setInputCloud(input_, indices_);
    }

    // Check if the search tree is initialized with the same input cloud
    if (tree_->getInputCloud() != input_) {
      if (tree_->getInputCloud() == nullptr) {
        tree_->setInputCloud(input_, indices_);
      } else {
        PCL_ERROR("[pcl::%s::initCompute] Input cloud for serach not "
                  "idential to "
                  "input!\n",
                  getClassName().c_str());
        deinitCompute();
        return (false);
      }
    }

    // Do a fast check to see if the search parameters are well defined
    if (search_radius_ != 0.0) {
      if (k_ != 0) {
        PCL_ERROR("[pcl::%s::compute] ", getClassName().c_str());
        PCL_ERROR("Both radius (%f) and K (%d) defined! ", search_radius_, k_);
        PCL_ERROR("Set one of them to zero first and then re-run compute "
                  "().\n");
        // Cleanup
        deinitCompute();
        return (false);
      } else // Use the radiusSearch () function
      {
        search_parameter_ = search_radius_;
        // Declare the search locator definition
        search_method_ = [this](const PointCloud &cloud, int index,
                                pcl::Indices &k_indices,
                                std::vector<float> &k_distances) {
          return tree_->radiusSearch(cloud, index, search_parameter_, k_indices,
                                     k_distances, 0);
        };
      }
    } else {
      if (k_ != 0) // Use the nearestKSearch () function
      {
        search_parameter_ = k_;
        // Declare the search locator definition
        search_method_ = [this](const PointCloud &cloud, int index,
                                pcl::Indices &k_indices,
                                std::vector<float> &k_distances) {
          return tree_->nearestKSearch(cloud, index, (int)search_parameter_,
                                       k_indices, k_distances);
        };
      } else {
        PCL_ERROR("[pcl::%s::compute] Neither radius nor K defined! ",
                  getClassName().c_str());
        PCL_ERROR("Set one of them to a positive number first and then re-run "
                  "compute ().\n");
        // Cleanup
        deinitCompute();
        return (false);
      }
    }

    // Configure the comparator
    compare_->setInputCloud(input_);
    compare_->setInputNormals(normals_);
    compare_->setAngularThreshold(angular_threshold_);
    compare_->setDistanceThreshold(distance_threshold_);

    // Set the initial update interval
    next_update_ = initial_interval_;

    // Initialize the PCA object
    pca_->setInputCloud(input_);

    // Clear the output vectors
    num_pts_in_segment_.clear();
    model_coefficients_.clear();
    inlier_indices_.clear();
    centroids_.clear();

    return (true);
  }

  /** \brief This method grows a segment for the given seed point. And
   * returns the number of its points.
   * \param[in] initial_seed index of the point that will serve as the
   * seed point
   * \param[in] segment_number indicates which number this segment will
   * have
   */
  size_t growRegion(const size_t initial_seed, unsigned segment_number) {

    std::queue<size_t> seeds;
    seeds.push(initial_seed);

    const unsigned label_id = segment_number + 1; // to avoid 0 label

    labels_->points[initial_seed].label = label_id;

    size_t num_pts = 1;

    compare_->initializePlane(initial_seed);
    next_update_ = initial_interval_;

    IndicesPtr plane_indices(new Indices);
    plane_indices->push_back(initial_seed);

    // TODO: See if there is an omp parallel queue implementation
    while (!seeds.empty()) {
      size_t curr_seed = seeds.front();
      seeds.pop();

      for (size_t i_nghbr = 0; i_nghbr < point_neighbours_[curr_seed].size();
           i_nghbr++) {

        // Get the index of the neighbour point
        int index = point_neighbours_[curr_seed][i_nghbr];

        // Check if the point is already labeled
        if (labels_->points[index].label != 0)
          continue;

        if (!compare_->compare(index))
          // Point is not part of the plane
          continue;

        labels_->points[index].label = label_id;
        num_pts++;

        seeds.push(index);
        plane_indices->push_back(index);

      } // next neighbour

      // INFO: Update the plane coefficients
      if (num_pts >= next_update_) {
        pca_->setIndices(plane_indices);
        compare_->setPlaneNormal(pca_->getEigenVectors().col(2));
        compare_->setPlaneOrigin(pca_->getMean());
        next_update_ *= interval_factor_;
      }
    } // next seed

    return num_pts;
  }

  /** \brief Get a string representation of the name of this class. */
  inline const std::string getClassName() const {
    return ("PlanarRegionGrowing");
  }

    protected:
  /** \brief A pointer to the input normals */
  PointCloudNConstPtr normals_{nullptr};

  /** \brief A pointer to the input labels */
  PointCloudLPtr labels_{nullptr};

  /** \brief Threshold that tells if point have the same normal
   * orienation*/
  float angular_threshold_{pcl::deg2rad(25.0f)};

  /** \brief Threshold for how far point can ly from the plane*/
  float distance_threshold_{0.05f};

  /** \brief Minimum number of inliers required for a plane */
  unsigned min_inliers_{1000};

  /** \brief Initial update interval for the plane fitting */
  float initial_interval_{16.0f};

  /** \brief Factor by which the update interval in multiplied after each
   * plane fitting */
  float interval_factor_{1.5f};

  /** \brief Early stop factor for the region growing algorithm.*/
  float early_stop_factor_{0.3f};

  /** \brief A tree for fast neighbor search. */
  KdTreePtr tree_{nullptr}; // KdTreePtr(new KdTree(false));

  /** \brief The search method template for points. */
  SearchMethod search_method_{nullptr};

  /** \brief The actual search parameter (from either \a search_radius_ or
   * \a k_). */
  double search_parameter_{0.0};

  /** \brief The nearest neighbors search radius for each point. */
  double search_radius_{0.0};

  /** \brief The number of K nearest neighbors to use for each point. */
  int k_{0};

  /** \brief The PCA object used for plane fitting. */
  PCAPtr pca_ = std::make_shared<PCA>(new PCA());

  /** \brief A comparator for determining if a point belongs to the
   * current planes buy comparing its normal and distance to the plane.
   */
  PlaneComparatorPtr compare_{new PlaneComparator};

  /** \brief A vector of neibour indices for each point in the input
   * cloud. This is doen to avoid recomputing the neighbors for each point
   * in the
   */
  std::vector<pcl::Indices> point_neighbours_{};
  /** \brief A vector of model coefficients for each plane found in the
   * input cloud. */
  std::vector<pcl::ModelCoefficients> model_coefficients_{};

  /** \brief A vector of inlier indices for each detected plane. */
  std::vector<pcl::Indices> inlier_indices_{};

  /** \brief Tells how much points each segment contains. Used for
   * reserving memory. */
  std::vector<pcl::uindex_t> num_pts_in_segment_{};

  /** \brief A vector of centroids for each plane found in the input
   * cloud. The centroids are stored as Eigen::Vector4f to include the
   * homogeneous coordinate.
   */
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids_{};

  /** \brief Variable to store the value for the next update of the plane
   * coefficients. */
  float next_update_{0.0f};

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// TODO: Consider inheriting from comparator
// Though I am not sure what the benefits would be apart from overall
// legebility and integration in to pcl
template <typename PointT, typename PointNT>
class PlanarRegionGrowingComparator : public PCLBase<PointT> {
    public:
  using Ptr = std::shared_ptr<PlanarRegionGrowingComparator<PointT, PointNT>>;
  using ConstPtr =
      std::shared_ptr<const PlanarRegionGrowingComparator<PointT, PointNT>>;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  using PointCloudN = pcl::PointCloud<PointNT>;
  using PointCloudNPtr = typename PointCloudN::Ptr;
  using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

  using PCLBase<PointT>::setInputCloud;
  using PCLBase<PointT>::getInputCloud;

    protected:
  using PCLBase<PointT>::input_;

    public:
  PlanarRegionGrowingComparator() = default;
  ~PlanarRegionGrowingComparator() = default;

  /** \brief Compare a point with the plane defined by the plane normal
   * and origin.
   * \param[in] idx the index of the point to compare
   * \return true if the point is part of the plane, false otherwise
   */
  bool compare(int idx) {
    if (!init_has_been_called_) {
      initCompute();
    }
    if (!init_has_been_called_) {
      PCL_ERROR("[pcl::%s::compare] Initialization failed.\n",
                getClassName().c_str());
      return false;
    }

    //// Check if the absolute value of the angle between the point normal
    //// and the plane normal is less than the angular threshold
    // if (std::abs(plane_normal_.dot(normals_->at(idx).getNormalVector3fMap()))
    // >
    //     angular_threshold_) {
    //   return false; // Point normal is not aligned with the plane normal
    // }

    // Check if the distance of the point to the plane is less than the
    // distance threshold
    if ((input_->at(idx).getVector3fMap() -
         projectPointOnPlane(input_->at(idx).getVector3fMap()))
            .norm() > distance_threshold_) {
      return false; // Point is too far from the plane
    }

    return true;
  }

  /** \brief Initialize the plane normal and origin from the input point
   * cloud and normals.
   * \param[in] index the index of the point to use for initialization
   */
  void initializePlane(size_t index) {
    plane_origin_ = input_->at(index).getVector4fMap();
    plane_normal_ = normals_->at(index).getNormalVector3fMap();
  }

  /** \brief Set the input normals for the comparator.
   * \param[in] normals the input normal cloud
   */
  inline void
  setInputNormals(const typename pcl::PointCloud<PointNT>::ConstPtr &normals) {
    normals_ = normals;
  }

  /** \brief Get the input normals for the comparator. */
  inline typename pcl::PointCloud<PointNT>::ConstPtr getInputNormals() const {
    return normals_;
  }

  /** \brief Set the plane normal to be used for comparison.
   * \param[in] normal the plane normal
   */
  inline void setPlaneNormal(const Eigen::Vector3f &normal) {
    plane_normal_ = normal;
  }

  /** \brief Get the plane normal used for comparison. */
  inline Eigen::Vector3f getPlaneNormal() const { return plane_normal_; }

  /** \brief Set the plane origin to be used for comparison.
   * \param[in] origin the plane origin
   */
  inline void setPlaneOrigin(const Eigen::Vector4f &origin) {
    plane_origin_ = origin;
  }

  /** \brief Get the plane origin used for comparison.
   * \return the plane origin
   */
  inline Eigen::Vector4f getPlaneOrigin() const { return plane_origin_; }

  /** \brief Set the angular threshold for comparison.
   * \param[in] threshold the angular threshold in radians
   */
  inline void setAngularThreshold(float threshold) {
    angular_threshold_ = threshold;
  }

  /** \brief Get the angular threshold for comparison.
   * \return the angular threshold in radians
   */
  inline float getAngularThreshold() const { return angular_threshold_; }

  /** \brief Set the distance threshold for comparison.
   * \param[in] threshold the distance threshold in meters
   */
  inline void setDistanceThreshold(float threshold) {
    distance_threshold_ = threshold;
  }

  /** \brief Get the distance threshold for comparison.
   * \return the distance threshold in meters
   */
  inline float getDistanceThreshold() const { return distance_threshold_; }

  /** \brief Get the coefficients of the plane equation.
   * \return the coefficients of the plane equation as a vector of size 4
   */

  ModelCoefficients getModelCoefficients() const {
    ModelCoefficients coeff;
    coeff.values.resize(4);
    coeff.values[0] = plane_normal_[0];
    coeff.values[1] = plane_normal_[1];
    coeff.values[2] = plane_normal_[2];
    coeff.values[3] = -plane_normal_.dot(plane_origin_.head<3>());
    return coeff;
  }

    protected:
  bool initCompute() {
    if (!normals_) {
      PCL_ERROR("[pcl::%s::initCompute] No normals set.\n",
                getClassName().c_str());
      return false;
    }
    if (normals_->size() != input_->size()) {
      PCL_ERROR("[pcl::%s::initCompute] Normals size (%zu) does not match "
                "input size (%zu).\n",
                getClassName().c_str(), normals_->size(), input_->size());
      return false;
    }

    init_has_been_called_ = true;
    return true;
  }

  inline const std::string getClassName() const {
    return ("PlanarRegionGrowingComparator");
  }

  Eigen::Vector3f projectPointOnPlane(const Eigen::Vector3f point) const {
    Eigen::Vector3f projected_point;
    pcl::geometry::project(point, plane_origin_.head<3>(), plane_normal_,
                           projected_point);
    return projected_point;
  }

    protected:
  bool init_has_been_called_ = false;

  PointCloudNConstPtr normals_{nullptr};

  float angular_threshold_ = pcl::deg2rad(25.0f);
  float distance_threshold_ = 0.05f;

  Eigen::Vector3f plane_normal_ = Eigen::Vector3f::Zero();
  Eigen::Vector4f plane_origin_ = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
};
} // namespace pcl

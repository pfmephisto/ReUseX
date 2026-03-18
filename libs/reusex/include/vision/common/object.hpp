#pragma once
#include <opencv2/opencv.hpp>

#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>

namespace ReUseX::vision::common::object {

/// @brief Enumeration of supported detection object types.
/// Identifies the kind of data stored in a DetectionBox.
enum class ObjectType {
  UNKNOW = -1,        ///< Unknown or unclassified object type.
  POSITION = 0,       ///< Simple position/region of interest.
  POSE = 1,           ///< Human pose estimation result.
  OBB = 2,            ///< Oriented bounding box.
  SEGMENTATION = 3,   ///< Instance segmentation mask.
  DEPTH_ANYTHING = 4, ///< Depth map from DepthAnything model.
  DEPTH_PRO = 5,      ///< Depth map from DepthPro model.
  TRACK = 6,          ///< Multi-object tracking result.
  DETECTION = 7,      ///< Standard bounding box detection.
};

/// @brief Axis-aligned bounding box defined by (left, top, right, bottom)
/// coordinates.
struct Box {
  float left = 0.0f;   ///< Left edge x-coordinate.
  float top = 0.0f;    ///< Top edge y-coordinate.
  float right = 0.0f;  ///< Right edge x-coordinate.
  float bottom = 0.0f; ///< Bottom edge y-coordinate.

  Box() = default;

  /// @brief Construct a Box with explicit corner coordinates.
  /// @param l Left edge.
  /// @param t Top edge.
  /// @param r Right edge.
  /// @param b Bottom edge.
  Box(float l, float t, float r, float b);

  /// @brief Returns the width of the box.
  float width() const noexcept { return right - left; }

  /// @brief Returns the height of the box.
  float height() const noexcept { return bottom - top; }

  /// @brief Returns the x-coordinate of the box center.
  float center_x() const noexcept { return (left + right) / 2; }

  /// @brief Returns the y-coordinate of the box center.
  float center_y() const noexcept { return (top + bottom) / 2; }

  /// @brief Returns the area of the box.
  float area() const noexcept { return width() * height(); }

  friend std::ostream &operator<<(std::ostream &os, const Box &box);
};

/// @brief A single keypoint for pose estimation, with visibility flag.
struct PosePoint {
  float x = 0.0f;   ///< X-coordinate of the keypoint.
  float y = 0.0f;   ///< Y-coordinate of the keypoint.
  float vis = 0.0f; ///< Visibility/confidence score (0 = invisible, 1 =
                    ///< visible).

  PosePoint() = default;

  /// @brief Construct a PosePoint with position and visibility.
  PosePoint(float x, float y, float vis);

  PosePoint &operator=(const PosePoint &other);
  friend std::ostream &operator<<(std::ostream &os, const PosePoint &point);
};

/// @brief A set of keypoints representing a human or object pose.
struct Pose {
  std::vector<PosePoint> points; ///< Ordered list of pose keypoints.

  Pose &operator=(const Pose &other);
  friend std::ostream &operator<<(std::ostream &os, const Pose &pose);
};

/// @brief Oriented bounding box (OBB) parameterized by center, size, and
/// rotation angle.
struct Obb {
  float cx = 0.0f;    ///< Center x-coordinate.
  float cy = 0.0f;    ///< Center y-coordinate.
  float w = 0.0f;     ///< Width of the box.
  float h = 0.0f;     ///< Height of the box.
  float angle = 0.0f; ///< Rotation angle in degrees.

  Obb() = default;

  /// @brief Construct an Obb with center, size, and angle.
  Obb(float cx, float cy, float w, float h, float angle);

  Obb &operator=(const Obb &other);

  /// @brief Returns the area of the oriented bounding box.
  float area() const { return w * h; }

  friend std::ostream &operator<<(std::ostream &os, const Obb &obb);
};

/// @brief A raw binary segmentation map stored as pinned host memory.
/// Width must be a multiple of 8. Move-only (no copy).
struct SegmentMap {
  int width = 0;  ///< Width of the mask in pixels (must be % 8 == 0).
  int height = 0; ///< Height of the mask in pixels.
  unsigned char *data = nullptr; ///< Raw mask data (width * height bytes).

  /// @brief Allocate a SegmentMap of the given dimensions using pinned CUDA
  /// host memory.
  SegmentMap(int width, int height);

  virtual ~SegmentMap();

  /// Copy is disabled — use move semantics instead.
  SegmentMap(const SegmentMap &) = delete;
  SegmentMap &operator=(const SegmentMap &) = delete;

  /// @brief Move constructor — transfers ownership and resets source.
  SegmentMap(SegmentMap &&other) noexcept
      : width(std::exchange(other.width, 0)),
        height(std::exchange(other.height, 0)),
        data(std::exchange(other.data, nullptr)) {}

  /// @brief Move assignment — transfers ownership and resets source.
  SegmentMap &operator=(SegmentMap &&other) noexcept;
};

/// @brief Instance segmentation result backed by an OpenCV mask.
struct Segmentation {
  cv::Mat mask; ///< Binary or grayscale segmentation mask.

  /// @brief Retain only the largest connected component in the mask.
  void keep_largest_part();

  /// @brief Return a new Segmentation placed at the given (left, top) offset
  /// within a canvas of the specified dimensions.
  Segmentation align_to_left_top(int left, int top, int width,
                                 int height) const;

  Segmentation &operator=(const Segmentation &other);
};

/// @brief Depth map result with per-pixel depth values.
struct Depth {
  cv::Mat depth;         ///< CV_32F depth image.
  float fog_data = 0.0f; ///< Optional fog/haze estimate.

  /// @brief Returns the depth at pixel (x, y).
  float point_depth(int x, int y) const;

  /// @brief Returns the average depth over the entire map.
  float average_depth() const;

  /// @brief Returns the minimum depth value.
  float min_depth() const;

  /// @brief Returns the maximum depth value.
  float max_depth() const;

  /// @brief Returns the average depth within the region indicated by a
  /// segmentation mask.
  float area_average_depth(const cv::Mat &seg) const;

  /// @brief Returns the average depth within the given bounding box region.
  float area_average_depth(const Box &box) const;
};

/// @brief Multi-object tracking state for a single tracked instance.
struct Track {
  int track_id = -1; ///< Unique tracking identifier.

  /// @brief Optional history of pose keypoints from previous frames.
  std::optional<std::vector<Pose>> history_pose;

  /// @brief Trajectory trace as (x, y) positions from previous frames.
  std::vector<std::tuple<float, float>> track_trace;

  friend std::ostream &operator<<(std::ostream &os, const Track &track);
};

/// @brief Universal detection result container that holds a bounding box plus
/// optional enriched data (pose, OBB, segmentation, depth, tracking).
struct DetectionBox {
  ObjectType type = ObjectType::UNKNOW; ///< Type of this detection.
  Box box;                              ///< Axis-aligned bounding box.
  float score = 0.0f;                   ///< Confidence score in [0, 1].
  int class_id = -1;                    ///< Class index.
  std::string class_name;               ///< Human-readable class label.

  std::optional<Pose> pose; ///< Populated for POSE detections.
  std::optional<Obb> obb;   ///< Populated for OBB detections.
  std::optional<Segmentation>
      segmentation;           ///< Populated for SEGMENTATION detections.
  std::optional<Depth> depth; ///< Populated for DEPTH_* detections.
  std::optional<Track> track; ///< Populated for TRACK detections.

  friend std::ostream &operator<<(std::ostream &os, const DetectionBox &box);
};

/// @brief Convert a SegmentMap to an OpenCV Mat (zero-copy header wrapper).
/// @param map The shared SegmentMap to wrap.
/// @return A CV_8UC1 Mat pointing to the SegmentMap's data buffer.
cv::Mat segmentMapToMat(const std::shared_ptr<SegmentMap> &map);

/// @brief Convenience alias for a collection of DetectionBox results.
using DetectionBoxArray = std::vector<DetectionBox>;

} // namespace ReUseX::vision::common::object

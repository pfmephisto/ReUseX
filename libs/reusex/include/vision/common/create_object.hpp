#pragma once
#include "reusex/vision/common/object.hpp"

#include <string>
#include <vector>

namespace cv {
class Mat;
}

namespace reusex::vision::common::object {

/// @brief Create a standard detection box.
/// @param left   Left edge of the bounding box.
/// @param top    Top edge of the bounding box.
/// @param right  Right edge of the bounding box.
/// @param bottom Bottom edge of the bounding box.
/// @param score  Confidence score in [0, 1].
/// @param class_id   Numeric class index.
/// @param class_name Human-readable class label.
/// @return A DetectionBox with type DETECTION.
DetectionBox create_box(float left, float top, float right, float bottom,
                        float score, int class_id,
                        const std::string &class_name);

/// @brief Create a position/region-of-interest box (type POSITION).
/// @param left   Left edge.
/// @param top    Top edge.
/// @param right  Right edge.
/// @param bottom Bottom edge.
/// @param score  Confidence score.
/// @param class_id   Class index.
/// @param class_name Class label.
/// @return A DetectionBox with type POSITION.
DetectionBox create_position_box(float left, float top, float right,
                                 float bottom, float score, int class_id,
                                 const std::string &class_name);

/// @brief Create a simple tracking box without additional payload.
/// @param left   Left edge.
/// @param top    Top edge.
/// @param right  Right edge.
/// @param bottom Bottom edge.
/// @param score  Confidence score.
/// @param track_id Unique tracking identifier.
/// @param class_name Class label.
/// @return A DetectionBox with type TRACK.
DetectionBox create_track_box(float left, float top, float right, float bottom,
                              float score, int track_id,
                              const std::string &class_name);

/// @brief Create a tracking box that also carries pose keypoints.
DetectionBox create_track_box(float left, float top, float right, float bottom,
                              float score, int track_id,
                              const std::string &class_name,
                              const object::Pose &pose);

/// @brief Create a tracking box that also carries an oriented bounding box.
DetectionBox create_track_box(float left, float top, float right, float bottom,
                              float score, int track_id,
                              const std::string &class_name,
                              const object::Obb &obb);

/// @brief Create a tracking box that also carries a segmentation mask.
DetectionBox create_track_box(float left, float top, float right, float bottom,
                              float score, int track_id,
                              const std::string &class_name,
                              const object::Segmentation &seg);

/// @brief Create an oriented bounding box detection.
/// The axis-aligned bounding box is computed automatically from the OBB
/// parameters.
/// @param cx    Center x.
/// @param cy    Center y.
/// @param w     Width.
/// @param h     Height.
/// @param angle Rotation angle in degrees.
/// @param score Confidence score.
/// @param class_id   Class index.
/// @param class_name Class label.
/// @return A DetectionBox with type OBB.
DetectionBox create_obb_box(float cx, float cy, float w, float h, float angle,
                            float score, int class_id,
                            const std::string &class_name);

/// @brief Create a pose-estimation detection box.
/// @param left        Left edge.
/// @param top         Top edge.
/// @param right       Right edge.
/// @param bottom      Bottom edge.
/// @param pose_points Ordered list of pose keypoints.
/// @param score       Confidence score.
/// @param class_id    Class index.
/// @param class_name  Class label.
/// @return A DetectionBox with type POSE.
DetectionBox create_pose_box(float left, float top, float right, float bottom,
                             const std::vector<PosePoint> &pose_points,
                             float score, int class_id,
                             const std::string &class_name);

/// @brief Create a segmentation detection box with an associated mask.
/// @param left   Left edge.
/// @param top    Top edge.
/// @param right  Right edge.
/// @param bottom Bottom edge.
/// @param mask   Segmentation mask (will be cloned).
/// @param score  Confidence score.
/// @param class_id   Class index.
/// @param class_name Class label.
/// @return A DetectionBox with type SEGMENTATION.
DetectionBox create_segmentation_box(float left, float top, float right,
                                     float bottom, const cv::Mat &mask,
                                     float score, int class_id,
                                     const std::string &class_name);

/// @brief Create a depth detection from DepthPro output.
/// @param depth    CV_32F depth map (will be cloned).
/// @param fog_data Estimated fog/haze value.
/// @return A DetectionBox with type DEPTH_PRO.
DetectionBox create_depth_pro_box(const cv::Mat &depth, float fog_data);

/// @brief Create a depth detection from DepthAnything output.
/// @param depth CV_32F depth map (will be cloned).
/// @return A DetectionBox with type DEPTH_ANYTHING.
DetectionBox create_depth_anything_box(const cv::Mat &depth);

} // namespace reusex::vision::common::object

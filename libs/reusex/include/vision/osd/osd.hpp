#pragma once
#include "reusex/vision/common/object.hpp"

#include <opencv2/opencv.hpp>

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace reusex::vision::osd {

/// @brief Draw the base detection rectangle and label background for a single
/// box.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box providing coordinates and type.
/// @param color     BGR color for the rectangle.
/// @param thickness Line thickness in pixels.
void drawBaseInfoGeometry(cv::Mat &img, const common::object::DetectionBox &box,
                          const cv::Scalar &color, int thickness);

/// @brief Draw a dashed or solid rectangle indicating a position ROI.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box providing coordinates.
/// @param color     BGR color for the rectangle.
/// @param thickness Line thickness in pixels.
void drawPositionRectGeometry(cv::Mat &img,
                              const common::object::DetectionBox &box,
                              const cv::Scalar &color, int thickness);

/// @brief Draw pose skeleton connections between keypoints.
/// Supports both COCO 17-keypoint and Hand 21-keypoint formats.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box with pose data.
/// @param thickness Line thickness in pixels.
void drawPoseSkeleton(cv::Mat &img, const common::object::DetectionBox &box,
                      int thickness);

/// @brief Draw an oriented bounding box as a rotated rectangle.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box with OBB data.
/// @param thickness Line thickness in pixels.
void drawObbBox(cv::Mat &img, const common::object::DetectionBox &box,
                int thickness);

/// @brief Overlay a semi-transparent segmentation mask onto the image.
/// @param img Image to draw on (modified in-place).
/// @param box Detection box with segmentation mask.
void drawSegmentationMask(cv::Mat &img,
                          const common::object::DetectionBox &box);

/// @brief Draw the tracking trajectory trace for a tracked object.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box with track data.
/// @param font_size Font size used for the track ID label.
void drawTrackTrace(cv::Mat &img, const common::object::DetectionBox &box,
                    int font_size);

/// @brief Draw historical pose keypoints from previous frames.
/// @param img       Image to draw on (modified in-place).
/// @param box       Detection box with track history pose data.
/// @param thickness Line thickness in pixels.
void drawTrackHistoryPose(cv::Mat &img, const common::object::DetectionBox &box,
                          int thickness);

/// @brief Overlay a depth map as a colour-mapped image blended with the input.
/// @param img Image to draw on (modified in-place).
/// @param box Detection box with depth data.
void drawDepth(cv::Mat &img, const common::object::DetectionBox &box);

/// @brief Draw a polygon from a list of (x, y) vertices.
/// @param img       Image to draw on (modified in-place).
/// @param points    Ordered list of (x, y) vertices.
/// @param color     BGR line color.
/// @param thickness Line thickness in pixels.
void drawPolygon(cv::Mat &img,
                 const std::vector<std::tuple<float, float>> &points,
                 const cv::Scalar &color, int thickness);

/// @brief Minimal OSD that paints each segmentation mask with its class_id
/// color (no text labels).
/// @param img   Image to draw on (modified in-place).
/// @param boxes Detection results to visualize.
void make_labled_image(cv::Mat &img,
                       const common::object::DetectionBoxArray &boxes);

/// @brief Full on-screen display: draws bounding boxes, masks, poses, OBBs,
/// tracks, and text labels with automatic non-overlapping label placement.
/// @param img             Image to draw on (modified in-place).
/// @param boxes           Detection results to visualize.
/// @param osd_rect        When true, draws bounding rectangles and text labels.
/// @param font_scale_ratio Fraction of the shorter image dimension used to
///                        determine the base font size (default 0.04).
void osd(cv::Mat &img, const common::object::DetectionBoxArray &boxes,
         bool osd_rect = true, double font_scale_ratio = 0.04);

/// @brief Draw polygon overlays for a map of named regions.
/// @param img             Image to draw on (modified in-place).
/// @param points          Map of region name → polygon vertices.
/// @param color           BGR line and label color.
/// @param font_scale_ratio Base font size ratio.
void osd(
    cv::Mat &img,
    const std::unordered_map<std::string, std::vector<std::tuple<float, float>>>
        &points,
    const cv::Scalar &color = cv::Scalar(0, 255, 0),
    double font_scale_ratio = 0.04);

/// @brief Draw a single named polygon overlay.
/// @param img             Image to draw on (modified in-place).
/// @param fence_name      Label text drawn at the polygon centroid.
/// @param points          Polygon vertices as (x, y) pairs.
/// @param color           BGR line and label color.
/// @param font_scale_ratio Base font size ratio.
void osd(cv::Mat &img, const std::string &fence_name,
         const std::vector<std::tuple<float, float>> &points,
         const cv::Scalar &color = cv::Scalar(0, 255, 0),
         double font_scale_ratio = 0.04);

/// @brief Render a text string at an arbitrary (x, y) position.
/// @param img       Image to draw on (modified in-place).
/// @param position  (x, y) drawing origin.
/// @param text      UTF-8 string to render.
/// @param color     BGR text color.
/// @param font_size Font size in pixels.
void osd(cv::Mat &img, const std::tuple<float, float> &position,
         const std::string &text,
         const cv::Scalar &color = cv::Scalar(0, 255, 0), int font_size = 40);

} // namespace reusex::vision::osd

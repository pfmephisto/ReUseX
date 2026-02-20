#pragma once

#include <ReUseX/vision/common/object.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace ReUseX::vision {

// Basic drawing
void drawBaseInfoGeometry(cv::Mat &img, const object::DetectionBox &box,
                          const cv::Scalar &color, int thickness);
void drawPositionRectGeometry(cv::Mat &img, const object::DetectionBox &box,
                              const cv::Scalar &color, int thickness);

// Auxiliary drawing
void drawPoseSkeleton(cv::Mat &img, const object::DetectionBox &box,
                      int thickness);
void drawObbBox(cv::Mat &img, const object::DetectionBox &box, int thickness);
void drawSegmentationMask(cv::Mat &img, const object::DetectionBox &box);
void drawTrackTrace(cv::Mat &img, const object::DetectionBox &box,
                    int font_size);
void drawTrackHistoryPose(cv::Mat &img, const object::DetectionBox &box,
                          int thickness);
void drawDepth(cv::Mat &img, const object::DetectionBox &box);
void drawPolygon(cv::Mat &img,
                 const std::vector<std::tuple<float, float>> &points,
                 const cv::Scalar &color, int thickness);

// Main OSD function
void osd(cv::Mat &img, const object::DetectionBoxArray &boxes,
         bool osd_rect = true, double font_scale_ratio = 0.04);

// Polygon overload
void osd(
    cv::Mat &img,
    const std::unordered_map<std::string, std::vector<std::tuple<float, float>>>
        &points,
    const cv::Scalar &color = cv::Scalar(0, 255, 0),
    double font_scale_ratio = 0.04);
void osd(cv::Mat &img, const std::string &fence_name,
         const std::vector<std::tuple<float, float>> &points,
         const cv::Scalar &color = cv::Scalar(0, 255, 0),
         double font_scale_ratio = 0.04);
void osd(cv::Mat &img, const std::tuple<float, float> &position,
         const std::string &text,
         const cv::Scalar &color = cv::Scalar(0, 255, 0), int font_size = 40);
} // namespace ReUseX::vision

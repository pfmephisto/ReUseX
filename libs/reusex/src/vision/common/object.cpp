#include <iostream>
#include <string>
#include <vector>
#include "vision/common/object.hpp"
#include "vision/tensor_rt/common/check.hpp"

namespace ReUseX::vision::common::object {

static std::string ObjectTypeToString(ObjectType type) {
  switch (type) {
  case ObjectType::detection:
    return "DETECTION";
  case ObjectType::pose:
    return "POSE";
  case ObjectType::obb:
    return "OBB";
  case ObjectType::segmentation:
    return "SEGMENTATION";
  case ObjectType::track:
    return "TRACK";
  case ObjectType::depth_anything:
    return "DEPTH_ANYTHING";
  case ObjectType::depth_pro:
    return "DEPTH_PRO";
  case ObjectType::position:
    return "POSITION";
  case ObjectType::unknown:
  default:
    return "UNKNOWN";
  }
}

Box::Box(float l, float t, float r, float b)
    : left(l), top(t), right(r), bottom(b) {}

std::ostream &operator<<(std::ostream &os, const Box &box) {
  os << "{ \"left\": " << box.left << ", \"top\": " << box.top
     << ", \"right\": " << box.right << ", \"bottom\": " << box.bottom << " }";
  return os;
}

PosePoint::PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}

std::ostream &operator<<(std::ostream &os, const PosePoint &point) {
  os << "{ \"x\": " << point.x << ", \"y\": " << point.y
     << ", \"vis\": " << point.vis << " }";
  return os;
}

PosePoint &PosePoint::operator=(const PosePoint &other) {
  if (this == &other)
    return *this;
  this->x = other.x;
  this->y = other.y;
  this->vis = other.vis;
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Pose &pose) {
  os << "[";
  for (size_t i = 0; i < pose.points.size(); ++i) {
    os << pose.points[i];
    if (i < pose.points.size() - 1)
      os << ", ";
  }
  os << "]";
  return os;
}

Pose &Pose::operator=(const Pose &other) {
  if (this == &other)
    return *this;
  this->points = other.points;
  return *this;
}

Obb::Obb(float cx, float cy, float w, float h, float angle)
    : cx(cx), cy(cy), w(w), h(h), angle(angle) {}

Obb &Obb::operator=(const Obb &other) {
  if (this == &other)
    return *this;
  this->cx = other.cx;
  this->cy = other.cy;
  this->w = other.w;
  this->h = other.h;
  this->angle = other.angle;
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Obb &obb) {
  os << "{ \"cx\": " << obb.cx << ", \"cy\": " << obb.cy << ", \"w\": " << obb.w
     << ", \"h\": " << obb.h << ", \"angle\": " << obb.angle << " }";
  return os;
}

SegmentMap::SegmentMap(int width, int height) {
  this->width = width;
  this->height = height;
  checkRuntime(cudaMallocHost(&this->data, width * height));
}

SegmentMap::~SegmentMap() {
  if (this->data) {
    checkRuntime(cudaFreeHost(this->data));
    this->data = nullptr;
  }
  this->width = 0;
  this->height = 0;
}

SegmentMap &SegmentMap::operator=(SegmentMap &&other) noexcept {
  if (this != &other) {
    if (this->data) {
      checkRuntime(cudaFreeHost(this->data));
    }
    width = std::exchange(other.width, 0);
    height = std::exchange(other.height, 0);
    data = std::exchange(other.data, nullptr);
  }
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Track &track) {
  os << "{ \"track_id\": " << track.track_id << ", \"trace\": [";
  for (size_t i = 0; i < track.track_trace.size(); ++i) {
    os << "{ \"x\": " << std::get<0>(track.track_trace[i])
       << ", \"y\": " << std::get<1>(track.track_trace[i]) << " }";
    if (i < track.track_trace.size() - 1)
      os << ", ";
  }
  os << "] }";
  return os;
}

float Depth::point_depth(int x, int y) const {
  if (depth.empty() || y < 0 || y >= depth.rows || x < 0 || x >= depth.cols)
    return 0.0f;
  return depth.at<float>(y, x);
}

float Depth::average_depth() const {
  if (depth.empty())
    return 0.0f;
  return static_cast<float>(cv::mean(depth)[0]);
}

float Depth::min_depth() const {
  if (depth.empty())
    return 0.0f;
  double min_val;
  cv::minMaxLoc(depth, &min_val, nullptr, nullptr, nullptr);
  return static_cast<float>(min_val);
}

float Depth::max_depth() const {
  if (depth.empty())
    return 0.0f;
  double max_val;
  cv::minMaxLoc(depth, nullptr, &max_val, nullptr, nullptr);
  return static_cast<float>(max_val);
}

float Depth::area_average_depth(const cv::Mat &seg) const {
  if (depth.empty() || seg.empty())
    return 0.0f;
  cv::Mat masked_depth;
  depth.copyTo(masked_depth, seg);
  float sum_depth = cv::sum(masked_depth)[0];
  int area = cv::countNonZero(seg);
  return area > 0 ? sum_depth / area : 0.0f;
}

float Depth::area_average_depth(const Box &box) const {
  if (depth.empty())
    return 0.0f;
  cv::Rect roi(cv::Point(box.left, box.top), cv::Point(box.right, box.bottom));
  roi &= cv::Rect(0, 0, depth.cols, depth.rows);
  if (roi.area() == 0)
    return 0.0f;
  return static_cast<float>(cv::mean(depth(roi))[0]);
}

Segmentation &Segmentation::operator=(const Segmentation &other) {
  if (this == &other)
    return *this;
  this->mask = other.mask.clone();
  return *this;
}

void Segmentation::keep_largest_part() {
  if (mask.empty())
    return;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty())
    return;
  auto max_contour = std::max_element(
      contours.begin(), contours.end(),
      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });
  cv::Mat new_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
  cv::drawContours(new_mask, std::vector<std::vector<cv::Point>>{*max_contour},
                   -1, cv::Scalar(255), cv::FILLED);
  mask = new_mask;
}

Segmentation Segmentation::align_to_left_top(int left, int top, int width,
                                             int height) const {
  object::Segmentation aligned_seg;
  cv::Mat aligned_mask = cv::Mat::zeros(height, width, mask.type());
  if (mask.empty())
    return aligned_seg;
  int x_offset = std::max(0, left);
  int y_offset = std::max(0, top);
  int copy_width = std::min(mask.cols, width - x_offset);
  int copy_height = std::min(mask.rows, height - y_offset);
  if (copy_width > 0 && copy_height > 0) {
    cv::Rect src_roi(0, 0, copy_width, copy_height);
    cv::Rect dst_roi(x_offset, y_offset, copy_width, copy_height);
    mask(src_roi).copyTo(aligned_mask(dst_roi));
  }
  aligned_seg.mask = aligned_mask;
  return aligned_seg;
}

std::ostream &operator<<(std::ostream &os, const DetectionBox &box) {
  os << "{";
  os << " \"type\": \"" << ObjectTypeToString(box.type) << "\""
     << ", \"class_id\": " << box.class_id << ", \"class_name\": \""
     << box.class_name << "\""
     << ", \"score\": " << box.score << ", \"box\": " << box.box;
  if (box.pose.has_value())
    os << ", \"pose\": " << box.pose.value();
  if (box.obb.has_value())
    os << ", \"obb\": " << box.obb.value();
  if (box.track.has_value())
    os << ", \"track\": " << box.track.value();
  if (box.segmentation.has_value()) {
    const auto &mask = box.segmentation.value().mask;
    os << ", \"segmentation\": { \"width\": " << mask.cols
       << ", \"height\": " << mask.rows << " }";
  }
  if (box.depth.has_value()) {
    const auto &depth = box.depth.value().depth;
    os << ", \"depth\": { \"width\": " << depth.cols
       << ", \"height\": " << depth.rows << " }";
  }
  os << " }";
  return os;
}

cv::Mat segment_map_to_mat(const std::shared_ptr<SegmentMap> &map) {
  if (map->data == nullptr || map->width <= 0 || map->height <= 0)
    return cv::Mat();
  return cv::Mat(map->height, map->width, CV_8UC1, map->data);
}

} // namespace ReUseX::vision::common::object

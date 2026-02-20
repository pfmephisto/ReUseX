#include <ReUseX/vision/common/object.hpp>
#include <iostream>
#include <string>
#include <vector>

namespace ReUseX::vision::object {
std::string ObjectTypeToString(ObjectType type) {
  switch (type) {
  case ObjectType::DETECTION:
    return "DETECTION";
  case ObjectType::POSE:
    return "POSE";
  case ObjectType::OBB:
    return "OBB";
  case ObjectType::SEGMENTATION:
    return "SEGMENTATION";
  case ObjectType::TRACK:
    return "TRACK";
  case ObjectType::DEPTH_ANYTHING:
    return "DEPTH_ANYTHING";
  case ObjectType::DEPTH_PRO:
    return "DEPTH_PRO";
  case ObjectType::POSITION:
    return "POSITION";
  case ObjectType::UNKNOW:
  default:
    return "UNKNOW";
  }
}

// Implementation of Box constructor
Box::Box(float l, float t, float r, float b)
    : left(l), top(t), right(r), bottom(b) {}

// Overload of Box output stream operator, for printing
std::ostream &operator<<(std::ostream &os, const Box &box) {
  os << "{ \"left\": " << box.left << ", \"top\": " << box.top
     << ", \"right\": " << box.right << ", \"bottom\": " << box.bottom << " }";
  return os;
}

// Implementation of PosePoint constructor
PosePoint::PosePoint(float x, float y, float vis) : x(x), y(y), vis(vis) {}

// Overload of PosePoint output stream operator
std::ostream &operator<<(std::ostream &os, const PosePoint &point) {
  os << "{ \"x\": " << point.x << ", \"y\": " << point.y
     << ", \"vis\": " << point.vis << " }";
  return os;
}

PosePoint &PosePoint::operator=(const PosePoint &other) {
  if (this == &other) {
    return *this;
  }
  this->x = other.x;
  this->y = other.y;
  this->vis = other.vis;

  // 3. Return a reference to the current object to allow for chained
  // assignments (e.g., a = b = c).
  return *this;
}

// Overload of Pose output stream operator
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
  if (this == &other) {
    return *this;
  }
  this->points = other.points;
  return *this;
}

// Implementation of Obb constructor
Obb::Obb(float cx, float cy, float w, float h, float angle)
    : cx(cx), cy(cy), w(w), h(h), angle(angle) {}

Obb &Obb::operator=(const Obb &other) {
  if (this == &other) {
    return *this;
  }
  this->cx = other.cx;
  this->cy = other.cy;
  this->w = other.w;
  this->h = other.h;
  this->angle = other.angle;

  // 3. Return a reference to the current object to allow for chained
  // assignments (e.g., a = b = c).
  return *this;
}

// Overload of Obb output stream operator
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
  // Prevent self-assignment (though unlikely with &&)
  if (this != &other) {
    // Free existing resource first
    if (this->data) {
      checkRuntime(cudaFreeHost(this->data));
    }

    // Transfer ownership from 'other'
    width = std::exchange(other.width, 0);
    height = std::exchange(other.height, 0);
    data = std::exchange(other.data, nullptr);
  }
  return *this;
}

// Overload of Track output stream operator
std::ostream &operator<<(std::ostream &os, const Track &track) {
  os << "{ \"track_id\": " << track.track_id << ", \"trace\": [";
  for (size_t i = 0; i < track.track_trace.size(); ++i) {
    // std::get<index> is the correct way to access tuple elements
    os << "{ \"x\": " << std::get<0>(track.track_trace[i])
       << ", \"y\": " << std::get<1>(track.track_trace[i]) << " }";
    if (i < track.track_trace.size() - 1)
      os << ", ";
  }
  os << "] }";
  return os;
}

float Depth::point_depth(int x, int y) const {
  if (depth.empty() || y < 0 || y >= depth.rows || x < 0 || x >= depth.cols) {
    return 0.0f; // Boundary check to prevent out-of-bounds access
  }
  // Assume depth map is CV_32F (32-bit float)
  return depth.at<float>(y, x);
}

float Depth::average_depth() const {
  if (depth.empty())
    return 0.0f;
  // cv::mean returns a Scalar, we usually take the mean of the first channel
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
  // Use mask to calculate the average depth of a specific region
  cv::Mat masked_depth;
  depth.copyTo(masked_depth, seg); // Only copy regions where mask is nonzero

  float sum_depth = cv::sum(masked_depth)[0];
  int area = cv::countNonZero(seg);
  return area > 0 ? sum_depth / area : 0.0f;
}

float Depth::area_average_depth(const Box &box) const {
  if (depth.empty())
    return 0.0f;
  // Define region of interest (ROI)
  cv::Rect roi(cv::Point(box.left, box.top), cv::Point(box.right, box.bottom));
  // Intersect ROI with image boundary to ensure ROI does not exceed image range
  roi &= cv::Rect(0, 0, depth.cols, depth.rows);
  if (roi.area() == 0)
    return 0.0f;
  // Directly calculate the mean value of the ROI area
  return static_cast<float>(cv::mean(depth(roi))[0]);
}

Segmentation &Segmentation::operator=(const Segmentation &other) {
  if (this == &other) {
    return *this;
  }
  this->mask = other.mask.clone();
  return *this;
}

void Segmentation::keep_largest_part() {
  if (mask.empty())
    return;

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty())
    return;

  // Find the largest contour
  auto max_contour = std::max_element(
      contours.begin(), contours.end(),
      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

  // Create a new mask, keeping only the largest contour
  cv::Mat new_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
  cv::drawContours(new_mask, std::vector<std::vector<cv::Point>>{*max_contour},
                   -1, cv::Scalar(255), cv::FILLED);
  mask = new_mask;
}

Segmentation Segmentation::align_to_left_top(int left, int top, int width,
                                             int height) const {
  object::Segmentation aligned_seg;
  // The original mask is relative to left, top
  // Now we need to create a new mask, size width x
  // height, and place the original mask in the new mask
  cv::Mat aligned_mask = cv::Mat::zeros(height, width, mask.type());
  if (mask.empty())
    return aligned_seg;
  // Calculate placement position
  int x_offset = std::max(0, left);
  int y_offset = std::max(0, top);
  // Calculate the valid area of the original mask in the new mask
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

//================================================================================
// Modified: Overload of DetectionBox output stream operator
// This is the most important modification. It no longer uses a rigid switch structure,
// Instead, it prints all required fields first, then checks each optional member for existence.
// This approach is more flexible and can accurately reflect any combination of data.
//================================================================================
std::ostream &operator<<(std::ostream &os, const DetectionBox &box) {
  os << "{";

  // --- Print core/required fields ---
  os << " \"type\": \"" << ObjectTypeToString(box.type) << "\""
     << ", \"class_id\": " << box.class_id << ", \"class_name\": \""
     << box.class_name << "\""
     << ", \"score\": " << box.score << ", \"box\": " << box.box;

  // --- Check and print optional fields one by one ---
  if (box.pose.has_value()) {
    os << ", \"pose\": " << box.pose.value();
  }
  if (box.obb.has_value()) {
    os << ", \"obb\": " << box.obb.value();
  }
  if (box.track.has_value()) {
    os << ", \"track\": " << box.track.value();
  }
  if (box.segmentation.has_value()) {
    // Directly printing the entire mask matrix is impractical, so we only print its size information
    const auto &mask = box.segmentation.value().mask;
    os << ", \"segmentation\": { \"width\": " << mask.cols
       << ", \"height\": " << mask.rows << " }";
  }
  if (box.depth.has_value()) {
    // Similarly, only print the size information of the depth map
    const auto &depth = box.depth.value().depth;
    os << ", \"depth\": { \"width\": " << depth.cols
       << ", \"height\": " << depth.rows << " }";
  }

  os << " }";
  return os;
}

cv::Mat segmentMapToMat(const std::shared_ptr<SegmentMap> &map) {
  // Check if input data is valid
  if (map->data == nullptr || map->width <= 0 || map->height <= 0) {
    // If SegmentMap is invalid, return an empty cv::Mat
    return cv::Mat();
  }

  // Use SegmentMap's data pointer, height, and width to create a cv::Mat object header.
  // CV_8UC1 means this is an 8-bit unsigned single-channel image, which is usually the format for segmentation masks.
  // This cv::Mat constructor does not copy data, but directly uses the provided pointer.
  return cv::Mat(map->height, map->width, CV_8UC1, map->data);
}

} // namespace ReUseX::vision::object

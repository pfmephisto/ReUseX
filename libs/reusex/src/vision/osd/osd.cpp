#include <ReUseX/vision/osd/cvx_text.hpp>
#include <ReUseX/vision/osd/labelLayoutSolver.hpp>
#include <ReUseX/vision/osd/osd.hpp>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>

#include <pcl/common/colors.h>

constexpr int COCO_NUM_KEYPOINTS = 17;
constexpr int HAND_NUM_KEYPOINTS = 21;

namespace {

const std::vector<std::pair<int, int>> coco_pairs = {
    {0, 1}, {0, 2},  {1, 3},   {2, 4},   {0, 5},   {0, 6},
    {5, 6}, {5, 11}, {6, 12},  {11, 12}, {5, 7},   {7, 9},
    {6, 8}, {8, 10}, {11, 13}, {13, 15}, {12, 14}, {14, 16}};
const std::vector<std::pair<int, int>> hand_pairs = {
    {0, 1},   {0, 5},   {0, 9},   {0, 13},  {0, 17},  {5, 9},
    {9, 13},  {13, 17}, {1, 2},   {2, 3},   {3, 4},   {5, 6},
    {6, 7},   {7, 8},   {9, 10},  {10, 11}, {11, 12}, {13, 14},
    {14, 15}, {15, 16}, {17, 18}, {18, 19}, {19, 20}};

std::tuple<uint8_t, uint8_t, uint8_t> hsv2bgr(float h, float s, float v) {
  const int h_i = static_cast<int>(h * 6);
  const float f = h * 6 - h_i;
  const float p = v * (1 - s);
  const float q = v * (1 - f * s);
  const float t = v * (1 - (1 - f) * s);
  float r, g, b;
  switch (h_i) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  case 5:
    r = v;
    g = p;
    b = q;
    break;
  default:
    r = 1;
    g = 1;
    b = 1;
    break;
  }
  return {static_cast<uint8_t>(b * 255), static_cast<uint8_t>(g * 255),
          static_cast<uint8_t>(r * 255)};
}

std::tuple<uint8_t, uint8_t, uint8_t> random_color(int id) {
  auto c = pcl::GlasbeyLUT::at(id % pcl::GlasbeyLUT::size());
  return {c.r, c.g, c.b};
}

std::tuple<uint8_t, uint8_t, uint8_t> random_color(const std::string &label) {
  std::hash<std::string> hasher;
  return random_color(static_cast<int>(hasher(label) & 0x7FFFFFFF));
}

void overlay_mask(cv::Mat &image, const cv::Mat &mask,
                  const ::ReUseX::vision::common::object::Box &box,
                  const cv::Scalar &color, double alpha) {
  if (image.empty() || mask.empty())
    return;
  cv::Rect roi(cv::Point(box.left, box.top), cv::Point(box.right, box.bottom));
  roi &= cv::Rect(0, 0, image.cols, image.rows);
  if (roi.area() <= 0)
    return;
  cv::Mat image_roi = image(roi);
  cv::Mat resized_mask;
  cv::resize(mask, resized_mask, roi.size());
  cv::Mat color_patch(roi.size(), image.type(), color);
  cv::Mat weighted_color;
  cv::addWeighted(color_patch, alpha, image_roi, 1.0 - alpha, 0.0,
                  weighted_color);
  weighted_color.copyTo(image_roi, resized_mask);
}

std::tuple<float, float>
calculatePolygonCentroid(const std::vector<std::tuple<float, float>> &pts) {
  if (pts.empty())
    return {0.0f, 0.0f};
  float sum_x = 0, sum_y = 0;
  for (auto &p : pts) {
    sum_x += std::get<0>(p);
    sum_y += std::get<1>(p);
  }
  return {sum_x / pts.size(), sum_y / pts.size()};
}

std::filesystem::path getFontPath() {
  const std::vector<std::string> preferred_fonts = {
      "DejaVuSans.ttf",
      "DejaVuSans-Bold.ttf",
      "DejaVuSansMNerdFont-Regular.ttf",
      "LiberationSans-Regular.ttf",
      "Arial.ttf",
      "arial.ttf",
      "Helvetica.ttf",
      "FreeSans.ttf",
      "NotoSans-Regular.ttf"};

  std::vector<std::filesystem::path> search_dirs;

  if (const char *fontconfig_path = std::getenv("FONTCONFIG_PATH"))
    search_dirs.push_back(std::filesystem::path(fontconfig_path));
  if (const char *fc_file = std::getenv("FONTCONFIG_FILE")) {
    auto parent = std::filesystem::path(fc_file).parent_path();
    if (!parent.empty())
      search_dirs.push_back(parent);
  }

  search_dirs.push_back(".");

  if (const char *nix_profile = std::getenv("NIX_PROFILE"))
    search_dirs.push_back(std::filesystem::path(nix_profile) / "share" /
                          "fonts");
  search_dirs.push_back("/run/current-system/sw/share/fonts");
  if (const char *home = std::getenv("HOME")) {
    search_dirs.push_back(std::filesystem::path(home) / ".nix-profile" /
                          "share" / "fonts");
    search_dirs.push_back(std::filesystem::path(home) / ".local" / "share" /
                          "fonts");
  }

  search_dirs.push_back("/usr/share/fonts");
  search_dirs.push_back("/usr/local/share/fonts");
  search_dirs.push_back("/Library/Fonts");
  search_dirs.push_back("/System/Library/Fonts");
  if (const char *home = std::getenv("HOME"))
    search_dirs.push_back(std::filesystem::path(home) / "Library" / "Fonts");
  if (const char *windir = std::getenv("WINDIR"))
    search_dirs.push_back(std::filesystem::path(windir) / "Fonts");
  search_dirs.push_back("C:\\Windows\\Fonts");

  for (const auto &dir : search_dirs) {
    if (!std::filesystem::exists(dir))
      continue;
    for (const auto &font : preferred_fonts) {
      auto font_path = dir / font;
      if (std::filesystem::exists(font_path))
        return font_path;
      try {
        for (const auto &entry : std::filesystem::recursive_directory_iterator(
                 dir,
                 std::filesystem::directory_options::skip_permission_denied)) {
          if (entry.is_regular_file() && entry.path().filename() == font)
            return entry.path();
        }
      } catch (const std::filesystem::filesystem_error &) {
        continue;
      }
    }
  }

  // If no font found, return empty path - we'll handle gracefully
  return std::filesystem::path();
}

// Lazy initialization - only create text renderer when first used
static ReUseX::vision::osd::CvxText& getTextRenderer() {
  static std::unique_ptr<ReUseX::vision::osd::CvxText> text_renderer;
  static std::once_flag init_flag;

  std::call_once(init_flag, []() {
    try {
      auto font_path = getFontPath();
      if (!font_path.empty()) {
        text_renderer = std::make_unique<ReUseX::vision::osd::CvxText>(font_path.string().c_str());
      }
    } catch (const std::exception& e) {
      // Font loading failed - text renderer will remain null
      std::cerr << "Warning: Could not initialize text renderer: " << e.what() << std::endl;
    }
  });

  if (!text_renderer) {
    throw std::runtime_error(
        "Font file not found. Please install DejaVu Sans, Liberation Sans, or "
        "ensure a TrueType font is available.");
  }

  return *text_renderer;
}

} // namespace

namespace ReUseX::vision::osd {

void drawBaseInfoGeometry(cv::Mat &img, const common::object::DetectionBox &box,
                          const cv::Scalar &color, int thickness) {
  cv::Rect rect(cv::Point(box.box.left, box.box.top),
                cv::Point(box.box.right, box.box.bottom));
  cv::rectangle(img, rect, color, thickness);
}

void drawPositionRectGeometry(cv::Mat &img,
                              const common::object::DetectionBox &box,
                              const cv::Scalar &color, int thickness) {
  cv::Rect rect(cv::Point(box.box.left, box.box.top),
                cv::Point(box.box.right, box.box.bottom));
  cv::rectangle(img, rect, color, thickness);
}

void drawPoseSkeleton(cv::Mat &img, const common::object::DetectionBox &box,
                      int thickness) {
  if (box.type == common::object::ObjectType::TRACK || !box.pose)
    return;
  const auto &points = box.pose->points;
  const auto *pairs = (points.size() == COCO_NUM_KEYPOINTS)   ? &coco_pairs
                      : (points.size() == HAND_NUM_KEYPOINTS) ? &hand_pairs
                                                              : nullptr;
  if (!pairs)
    return;
  for (const auto &pair : *pairs) {
    const auto &p1 = points[pair.first];
    const auto &p2 = points[pair.second];
    if (p1.vis > 0 && p2.vis > 0) {
      auto c = random_color(static_cast<int>(&pair - &(*pairs)[0] + 100));
      cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y),
               {(double)std::get<0>(c), (double)std::get<1>(c),
                (double)std::get<2>(c)},
               thickness);
    }
  }
}

void drawObbBox(cv::Mat &img, const common::object::DetectionBox &box,
                int thickness) {
  if (box.type == common::object::ObjectType::TRACK || !box.obb)
    return;
  auto c = random_color(box.class_name);
  cv::RotatedRect rRect(cv::Point2f(box.obb->cx, box.obb->cy),
                        cv::Size2f(box.obb->w, box.obb->h), box.obb->angle);
  cv::Point2f vertices[4];
  rRect.points(vertices);
  for (int i = 0; i < 4; i++)
    cv::line(img, vertices[i], vertices[(i + 1) % 4],
             {(double)std::get<0>(c), (double)std::get<1>(c),
              (double)std::get<2>(c)},
             thickness);
}

void drawSegmentationMask(cv::Mat &img,
                          const common::object::DetectionBox &box) {
  if (!box.segmentation || box.segmentation->mask.empty())
    return;
  auto c = random_color(box.class_name);
  overlay_mask(
      img, box.segmentation->mask, box.box,
      {(double)std::get<0>(c), (double)std::get<1>(c), (double)std::get<2>(c)},
      0.5);
}

void drawTrackHistoryPose(cv::Mat &img, const common::object::DetectionBox &box,
                          int thickness) {
  // Implementation reserved for future use
  (void)img;
  (void)box;
  (void)thickness;
}

void drawDepth(cv::Mat &img, const common::object::DetectionBox &box) {
  if (!box.depth || box.depth->depth.empty())
    return;
  cv::Mat norm, cmap;
  cv::normalize(box.depth->depth, norm, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::applyColorMap(norm, cmap, cv::COLORMAP_JET);
  cv::addWeighted(img, 0.5, cmap, 0.5, 0.0, img);
}

void drawTrackTrace(cv::Mat &img, const common::object::DetectionBox &box,
                    int font_size) {
  if (!box.track)
    return;
  auto c = random_color(box.track->track_id);
  cv::Scalar color((double)std::get<0>(c), (double)std::get<1>(c),
                   (double)std::get<2>(c));
  int th = std::max(1, font_size / 15);
  const auto &trace = box.track->track_trace;
  for (size_t i = 1; i < trace.size(); ++i) {
    cv::line(
        img, cv::Point(std::get<0>(trace[i - 1]), std::get<1>(trace[i - 1])),
        cv::Point(std::get<0>(trace[i]), std::get<1>(trace[i])), color, th);
  }
  std::string text = "ID:" + std::to_string(box.track->track_id);
  getTextRenderer().putText(img, text,
                        cv::Point(box.box.center_x(), box.box.center_y()),
                        color, font_size * 0.8);
}

void drawPolygon(cv::Mat &img,
                 const std::vector<std::tuple<float, float>> &points,
                 const cv::Scalar &color, int thickness) {
  if (points.size() < 2)
    return;
  std::vector<cv::Point> poly;
  for (const auto &p : points)
    poly.emplace_back(std::get<0>(p), std::get<1>(p));
  cv::polylines(img, poly, true, color, thickness);
}

static int calculateDynamicFontSize(int img_w, int img_h,
                                    const common::object::Box &box,
                                    double ratio) {
  int target_size =
      std::max(12, static_cast<int>(std::min(img_w, img_h) * ratio));
  return target_size;
}

void make_labled_image(cv::Mat &img,
                       const common::object::DetectionBoxArray &boxes) {
  spdlog::debug("OSD called with {} boxes", boxes.size());
  for (const auto &box : boxes) {
    if (!box.segmentation || box.segmentation->mask.empty())
      return;
    if (img.empty() || box.segmentation->mask.empty())
      return;
    cv::Rect roi(cv::Point(box.box.left, box.box.top),
                 cv::Point(box.box.right, box.box.bottom));
    roi &= cv::Rect(0, 0, img.cols, img.rows);
    if (roi.area() <= 0)
      return;
    double alpha = 0.5;
    cv::Mat image_roi = img(roi);
    cv::Mat resized_mask;
    cv::resize(box.segmentation->mask, resized_mask, roi.size());
    spdlog::debug("Class Id: {}, Class Name: {}", box.class_id, box.class_name);
    cv::Mat color_patch(roi.size(), img.type(), box.class_id);
    color_patch.copyTo(image_roi, resized_mask);
  }
}

void osd(cv::Mat &img, const common::object::DetectionBoxArray &boxes,
         bool osd_rect, double font_scale_ratio) {
  spdlog::debug("OSD called with {} boxes, osd_rect={}, font_scale_ratio={}",
                boxes.size(), osd_rect, font_scale_ratio);

  int height = img.rows, width = img.cols;
  const int PAD_X = 2;
  const int PAD_Y = 2;

  LabelLayoutSolver solver(
      width, height, [&](const std::string &txt, int fontSize) -> TextSize {
        int w, h, base;
        getTextRenderer().getTextSize(txt, fontSize, &w, &h, &base);
        return {w, h, base};
      });

  std::vector<cv::Scalar> label_colors;
  std::vector<std::string> label_texts;

  for (const auto &box : boxes) {
    if (box.type == common::object::ObjectType::DEPTH_PRO ||
        box.type == common::object::ObjectType::DEPTH_ANYTHING) {
      drawDepth(img, box);
      continue;
    }

    auto c = random_color(box.class_name);
    cv::Scalar color(std::get<0>(c), std::get<1>(c), std::get<2>(c));

    int base_font_size =
        calculateDynamicFontSize(width, height, box.box, font_scale_ratio);
    int thickness = std::max(1, base_font_size / 10);

    if (osd_rect) {
      if (box.type == common::object::ObjectType::POSITION)
        drawPositionRectGeometry(img, box, color, thickness);
      else
        drawBaseInfoGeometry(img, box, color, thickness);
    }

    drawSegmentationMask(img, box);
    drawPoseSkeleton(img, box, thickness);
    drawTrackHistoryPose(img, box, thickness);
    drawTrackTrace(img, box, base_font_size);
    drawObbBox(img, box, thickness);

    if (osd_rect && (box.box.bottom - box.box.top) >= 1) {
      std::string text;
      if (box.type == common::object::ObjectType::POSITION)
        text = box.class_name;
      else {
        std::ostringstream oss;
        oss << box.class_name << " " << std::fixed << std::setprecision(2)
            << box.score;
        text = oss.str();
      }
      solver.add(box.box.left, box.box.top, box.box.right, box.box.bottom, text,
                 base_font_size);
      label_colors.push_back(color);
      label_texts.push_back(text);
    }
  }

  solver.solve();
  auto results = solver.getResults();

  for (size_t i = 0; i < results.size() && i < label_colors.size(); ++i) {
    const auto &res = results[i];
    cv::Rect bg_rect(static_cast<int>(res.x), static_cast<int>(res.y),
                     res.width, res.height);
    bg_rect &= cv::Rect(0, 0, width, height);
    if (bg_rect.area() <= 0)
      continue;
    int text_x = bg_rect.x + PAD_X;
    int text_y = bg_rect.y + PAD_Y + res.textAscent;
    getTextRenderer().putText(img, label_texts[i], cv::Point(text_x, text_y),
                          label_colors[i], res.fontSize);
  }
}

void osd(
    cv::Mat &img,
    const std::unordered_map<std::string, std::vector<std::tuple<float, float>>>
        &points,
    const cv::Scalar &color, double font_scale_ratio) {
  int h = img.rows, w = img.cols;
  int fs = std::max(20, static_cast<int>(std::min(w, h) * font_scale_ratio));
  int th = std::max(1, fs / 7);
  for (const auto &[l, pts] : points) {
    if (pts.empty())
      continue;
    drawPolygon(img, pts, color, th);
    if (pts.size() >= 3) {
      auto c = calculatePolygonCentroid(pts);
      getTextRenderer().putText(img, l, cv::Point(std::get<0>(c), std::get<1>(c)),
                            color, fs);
    }
  }
}

void osd(cv::Mat &img, const std::string &fence_name,
         const std::vector<std::tuple<float, float>> &points,
         const cv::Scalar &color, double font_scale_ratio) {
  int h = img.rows, w = img.cols;
  int fs = std::max(20, static_cast<int>(std::min(w, h) * font_scale_ratio));
  int th = std::max(1, fs / 7);
  if (points.empty())
    return;
  drawPolygon(img, points, color, th);
  if (points.size() >= 3) {
    auto c = calculatePolygonCentroid(points);
    getTextRenderer().putText(img, fence_name,
                          cv::Point(std::get<0>(c), std::get<1>(c)), color, fs);
  }
}

void osd(cv::Mat &img, const std::tuple<float, float> &position,
         const std::string &text, const cv::Scalar &color, int font_size) {
  getTextRenderer().putText(img, text,
                        cv::Point(std::get<0>(position), std::get<1>(position)),
                        color, font_size);
}

} // namespace ReUseX::vision::osd

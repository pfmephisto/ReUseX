// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/insta360_x4.hpp"

#include "core/logging.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace reusex::io {

namespace {

// ─── Calibration constants
// ────────────────────────────────────────────────────
//
// Source: Hugin .pto template in docs/guides/Process walk through.md §Phase 5.
// Reference resolution: 5888×2944 (each fisheye circle ≈ 2941×2941 px).
//
// Lens model: equidistant fisheye (r = f·θ), no polynomial distortion
// correction (a=b=c=0 in the template). FoV ≈ 200° per lens.
//
// FOLLOW-UP: per-capture control-point re-optimisation (e.g. via RANSAC on
// matched keypoints) could improve seam accuracy on individual captures.
// The static calibration here matches the reference X4 rig geometry.

// Reference input height (= height of one fisheye half-square).
constexpr double kRefInH = 2944.0;

// Front lens — left half of dual image (second 'i' line in .pto):
//   d=-1468.6  → cx_full = 2944 - 1468.6 = 1475.4  (within left 0..2941 half)
//   e=0        → cy = 2944/2 = 1472
constexpr double kFrontCx = 1475.4;
constexpr double kFrontCy = 1472.0;
constexpr double kFrontYaw = 28.816356929132;
constexpr double kFrontPitch = 15.4753436799108;
constexpr double kFrontRoll = -9.67415383504551;

// Back lens — right half of dual image (first 'i' line in .pto):
//   d=+1468.6  → cx_full = 2944 + 1468.6 = 4412.6  → relative to right half:
//   1470.6 e=0        → cy = 1472
constexpr double kBackCx = 1470.6;
constexpr double kBackCy = 1472.0;
constexpr double kBackYaw = -151.183643070868;
constexpr double kBackPitch = -15.4753436799109;
constexpr double kBackRoll = 9.67415383504551;

// Equidistant focal length: r_px = f * theta  (theta in radians).
// Derived from X4 spec (≈200° FoV) and the fisheye circle radius:
//   r_max ≈ 1440 px, theta_max = 100° = 1.745 rad  →  f = 1440/1.745 ≈ 825
constexpr double kRefFocal = 825.0;
// Cut off slightly inside the full circle to avoid highly-distorted outer ring.
constexpr double kRefMaxRadius = 1440.0;

// Output dimensions matching the Hugin template: p w5422 h2711 (exact 2:1).
constexpr int kRefOutW = 5422;
constexpr int kRefOutH = 2711;

// ─── Rotation
// ─────────────────────────────────────────────────────────────────
//
// Hugin convention (Y-down, X-right, Z-forward):
//   R_cam2world = Ry(yaw) · Rx(pitch) · Rz(roll)
// We need R_world2cam = R_cam2world^T  to map world direction → camera frame.

cv::Matx33d make_world2cam(double yaw_deg, double pitch_deg, double roll_deg) {
  const double y = yaw_deg * (M_PI / 180.0);
  const double p = pitch_deg * (M_PI / 180.0);
  const double r = roll_deg * (M_PI / 180.0);

  const double cy = std::cos(y), sy = std::sin(y);
  const double cp = std::cos(p), sp = std::sin(p);
  const double cr = std::cos(r), sr = std::sin(r);

  const cv::Matx33d Ry(cy, 0, sy, 0, 1, 0, -sy, 0, cy);
  const cv::Matx33d Rx(1, 0, 0, 0, cp, -sp, 0, sp, cp);
  const cv::Matx33d Rz(cr, -sr, 0, sr, cr, 0, 0, 0, 1);

  return (Ry * Rx * Rz).t(); // transpose = world2cam
}

// ─── Per-lens remap maps
// ──────────────────────────────────────────────────────

struct LensMaps {
  cv::Mat map_x;  // CV_32F: source x coord in the full dual image
  cv::Mat map_y;  // CV_32F: source y coord
  cv::Mat weight; // CV_32F: cos(θ) from optical axis; 0 where not visible
};

// Build remap maps from an equirectangular output to one fisheye half.
//
// half_x_offset: pixel offset into the full dual image for this half
//   (0 for left/front, in_w/2 for right/back).
LensMaps build_lens_maps(int out_w, int out_h, double cx, double cy,
                         double focal, double max_r, const cv::Matx33d &R_w2c,
                         int half_x_offset) {
  LensMaps lm;
  lm.map_x.create(out_h, out_w, CV_32F);
  lm.map_y.create(out_h, out_w, CV_32F);
  lm.weight.create(out_h, out_w, CV_32F);

  const double inv_w = 1.0 / out_w;
  const double inv_h = 1.0 / out_h;

  for (int v = 0; v < out_h; ++v) {
    float *mx = lm.map_x.ptr<float>(v);
    float *my = lm.map_y.ptr<float>(v);
    float *mw = lm.weight.ptr<float>(v);

    // Equirect latitude (Y-down: top row = max positive lat = up)
    const double lat =
        M_PI * 0.5 - (static_cast<double>(v) + 0.5) * inv_h * M_PI;
    const double cos_lat = std::cos(lat);
    const double sin_lat = std::sin(lat);

    for (int u = 0; u < out_w; ++u) {
      // Equirect longitude: left edge = -π, right edge = +π
      const double lon =
          (static_cast<double>(u) + 0.5) * inv_w * 2.0 * M_PI - M_PI;

      // World direction (X=right, Y=down, Z=forward):
      //   wy = -sin(lat) so that positive lat (up) gives negative Y.
      const double wx = cos_lat * std::sin(lon);
      const double wy = -sin_lat;
      const double wz = cos_lat * std::cos(lon);

      // Transform to camera frame
      const double cx_d =
          R_w2c(0, 0) * wx + R_w2c(0, 1) * wy + R_w2c(0, 2) * wz;
      const double cy_d =
          R_w2c(1, 0) * wx + R_w2c(1, 1) * wy + R_w2c(1, 2) * wz;
      const double cz_d =
          R_w2c(2, 0) * wx + R_w2c(2, 1) * wy + R_w2c(2, 2) * wz;

      if (cz_d <= 0.0) {
        mx[u] = -1.f;
        my[u] = -1.f;
        mw[u] = 0.f;
        continue;
      }

      const double r_xy = std::sqrt(cx_d * cx_d + cy_d * cy_d);
      const double theta = std::atan2(r_xy, cz_d); // angle from optic axis
      const double r_px = focal * theta;           // equidistant projection

      if (r_px > max_r) {
        mx[u] = -1.f;
        my[u] = -1.f;
        mw[u] = 0.f;
        continue;
      }

      const double scale = (r_xy > 1e-9) ? (r_px / r_xy) : 0.0;
      mx[u] = static_cast<float>(cx + scale * cx_d) + half_x_offset;
      my[u] = static_cast<float>(cy + scale * cy_d);
      mw[u] =
          static_cast<float>(cz_d); // weight ∝ cos(θ): peak at axis, 0 at edge
    }
  }
  return lm;
}

// ─── Map cache (keyed by input height — the X4 rig geometry is fixed) ────────

struct CachedMaps {
  LensMaps front;
  LensMaps back;
  int out_w;
  int out_h;
};

std::unordered_map<int, CachedMaps> g_map_cache;
std::mutex g_cache_mu;

const CachedMaps &get_maps(int in_h, int in_w) {
  std::lock_guard<std::mutex> lk(g_cache_mu);
  auto it = g_map_cache.find(in_h);
  if (it != g_map_cache.end())
    return it->second;

  const double scale = static_cast<double>(in_h) / kRefInH;
  const int half_w = in_w / 2;
  const double focal = kRefFocal * scale;
  const double max_r = kRefMaxRadius * scale;

  // Output: scale reference dims, force exact 2:1.
  int out_w = static_cast<int>(std::round(kRefOutW * scale));
  if (out_w % 2 != 0)
    ++out_w;
  const int out_h = out_w / 2;

  CachedMaps cm;
  cm.out_w = out_w;
  cm.out_h = out_h;
  cm.front = build_lens_maps(
      out_w, out_h, kFrontCx * scale, kFrontCy * scale, focal, max_r,
      make_world2cam(kFrontYaw, kFrontPitch, kFrontRoll), 0);
  cm.back = build_lens_maps(
      out_w, out_h, kBackCx * scale, kBackCy * scale, focal, max_r,
      make_world2cam(kBackYaw, kBackPitch, kBackRoll), half_w);

  reusex::debug("insta360_x4: built remap maps {}x{} → {}x{}", in_w, in_h,
                out_w, out_h);
  return g_map_cache.emplace(in_h, std::move(cm)).first->second;
}

} // anonymous namespace

// ─── Public API ──────────────────────────────────────────────────────────────

bool is_insta360_dual_fisheye(const std::filesystem::path &path) {
  auto ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  return ext == ".insp";
}

cv::Mat stitch_insta360_x4(const cv::Mat &dual_fisheye) {
  if (dual_fisheye.empty())
    throw std::runtime_error("stitch_insta360_x4: empty input image");

  const int in_w = dual_fisheye.cols;
  const int in_h = dual_fisheye.rows;

  if (in_w != 2 * in_h || in_h < 100) {
    throw std::runtime_error("stitch_insta360_x4: expected dual-fisheye with "
                             "width == 2×height, got " +
                             std::to_string(in_w) + "×" + std::to_string(in_h));
  }

  // Ensure 3-channel BGR input
  cv::Mat src;
  if (dual_fisheye.channels() == 4)
    cv::cvtColor(dual_fisheye, src, cv::COLOR_BGRA2BGR);
  else if (dual_fisheye.channels() == 1)
    cv::cvtColor(dual_fisheye, src, cv::COLOR_GRAY2BGR);
  else
    src = dual_fisheye;

  const CachedMaps &maps = get_maps(in_h, in_w);

  // Warp each hemisphere into equirectangular
  cv::Mat img_front, img_back;
  cv::remap(src, img_front, maps.front.map_x, maps.front.map_y,
            cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  cv::remap(src, img_back, maps.back.map_x, maps.back.map_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  const int out_w = maps.out_w;
  const int out_h = maps.out_h;

  // Build binary coverage masks (CV_8U, 255 where weight > 0).
  cv::Mat mask_front_thresh, mask_back_thresh;
  cv::threshold(maps.front.weight, mask_front_thresh, 0.0, 255.0,
                cv::THRESH_BINARY);
  cv::threshold(maps.back.weight, mask_back_thresh, 0.0, 255.0,
                cv::THRESH_BINARY);
  cv::Mat mask_front, mask_back;
  mask_front_thresh.convertTo(mask_front, CV_8U);
  mask_back_thresh.convertTo(mask_back, CV_8U);

  // Gain compensation: align mean brightness between hemispheres so that
  // exposure differences don't create a visible step at the seam.
  {
    cv::UMat ufront, uback, umf, umb;
    img_front.copyTo(ufront);
    img_back.copyTo(uback);
    mask_front.copyTo(umf);
    mask_back.copyTo(umb);

    const std::vector<cv::Point> comp_corners = {cv::Point(0, 0),
                                                 cv::Point(0, 0)};
    const std::vector<cv::UMat> comp_images = {ufront, uback};
    const std::vector<cv::UMat> comp_masks = {umf, umb};

    auto compensator = cv::detail::ExposureCompensator::createDefault(
        cv::detail::ExposureCompensator::GAIN);
    compensator->feed(comp_corners, comp_images, comp_masks);
    compensator->apply(0, cv::Point(0, 0), img_front, mask_front);
    compensator->apply(1, cv::Point(0, 0), img_back, mask_back);
  }

  // Multi-band (Laplacian pyramid) blend: hides the seam by blending at
  // multiple frequency bands rather than a simple weighted average.
  // MultiBandBlender expects CV_16SC3 input.
  cv::Mat front16, back16;
  img_front.convertTo(front16, CV_16SC3);
  img_back.convertTo(back16, CV_16SC3);

  cv::detail::MultiBandBlender blender(/*try_gpu=*/false, /*num_bands=*/5);
  blender.prepare(cv::Rect(0, 0, out_w, out_h));
  blender.feed(front16, mask_front, cv::Point(0, 0));
  blender.feed(back16, mask_back, cv::Point(0, 0));

  cv::Mat result_s, result_mask;
  blender.blend(result_s, result_mask);

  cv::Mat result;
  result_s.convertTo(result, CV_8UC3);
  return result;
}

} // namespace reusex::io

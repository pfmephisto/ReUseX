// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>

namespace reusex::geometry {

/// Build a 4x4 Eigen affine (float) from a row-major double[16] array.
inline Eigen::Affine3f to_affine(const std::array<double, 16> &m) {
  Eigen::Matrix4f mat;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r, c) = static_cast<float>(m[r * 4 + c]);
  Eigen::Affine3f aff;
  aff.matrix() = mat;
  return aff;
}

/// Flatten a 4x4 Eigen affine (float) into a row-major double[16] array.
inline std::array<double, 16> to_array16(const Eigen::Affine3f &aff) {
  std::array<double, 16> m{};
  const Eigen::Matrix4f mat = aff.matrix();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m[r * 4 + c] = static_cast<double>(mat(r, c));
  return m;
}

/// Minimal se(3) exp/log used by the joint pose optimizer.
///
/// Twist ordering is xi = [omega (rotation, 3); v (translation, 3)], so that a
/// left-multiplied increment T <- exp(xi) * T moves a world point P by
/// (omega x P + v) to first order. This matches the Jacobian assembly in
/// JointPairwiseRegistration.
namespace se3 {

using Vector6d = Eigen::Matrix<double, 6, 1>;

/// Skew-symmetric matrix [w]_x such that [w]_x v == w x v.
inline Eigen::Matrix3d hat3(const Eigen::Vector3d &w) {
  Eigen::Matrix3d s;
  s << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return s;
}

/// Exponential map se(3) -> SE(3). xi = [omega; v].
inline Eigen::Matrix4d exp(const Vector6d &xi) {
  const Eigen::Vector3d w = xi.head<3>();
  const Eigen::Vector3d v = xi.tail<3>();
  const double theta = w.norm();
  const Eigen::Matrix3d W = hat3(w);
  Eigen::Matrix3d R;
  Eigen::Matrix3d V;
  if (theta < 1e-12) {
    R = Eigen::Matrix3d::Identity() + W;
    V = Eigen::Matrix3d::Identity() + 0.5 * W;
  } else {
    const double t2 = theta * theta;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    R = Eigen::Matrix3d::Identity() + (s / theta) * W + ((1.0 - c) / t2) * (W * W);
    V = Eigen::Matrix3d::Identity() + ((1.0 - c) / t2) * W +
        ((theta - s) / (t2 * theta)) * (W * W);
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = V * v;
  return T;
}

/// Logarithm map SE(3) -> se(3). Returns xi = [omega; v].
inline Vector6d log(const Eigen::Matrix4d &T) {
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  const double cos_theta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  Eigen::Vector3d w;
  if (theta < 1e-12) {
    w = 0.5 * Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0),
                              R(1, 0) - R(0, 1));
  } else {
    const double s = std::sin(theta);
    w = (theta / (2.0 * s)) * Eigen::Vector3d(R(2, 1) - R(1, 2),
                                              R(0, 2) - R(2, 0),
                                              R(1, 0) - R(0, 1));
  }
  const Eigen::Matrix3d W = hat3(w);
  Eigen::Matrix3d Vinv;
  if (theta < 1e-12) {
    Vinv = Eigen::Matrix3d::Identity() - 0.5 * W;
  } else {
    const double a =
        (1.0 / (theta * theta)) *
        (1.0 - (theta * std::sin(theta)) / (2.0 * (1.0 - std::cos(theta))));
    Vinv = Eigen::Matrix3d::Identity() - 0.5 * W + a * (W * W);
  }
  Vector6d xi;
  xi.head<3>() = w;
  xi.tail<3>() = Vinv * t;
  return xi;
}

} // namespace se3
} // namespace reusex::geometry

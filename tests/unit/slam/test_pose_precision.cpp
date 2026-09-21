// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/transform_utils.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

using namespace reusex::geometry;

namespace {

std::array<double, 16> make_pose(double tx, double ty, double tz,
                                 double angle_rad) {
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.rotate(Eigen::AngleAxisd(angle_rad,
                             Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  t.translation() = Eigen::Vector3d(tx, ty, tz);
  return to_array16(t);
}

} // namespace

// Verify that to_affine -> to_array16 is a lossless round-trip for double
// poses. Before the fix this failed: the float truncation in the old
// to_affine() produced differences of ~1e-7 (float epsilon * pose magnitude).
TEST_CASE("PosePrecision_RoundTrip_BitIdentical", "[pose][precision]") {
  // Georeferenced-scale coordinates: large enough to expose float-epsilon loss.
  const std::array<double, 16> original =
      make_pose(12.345678901234, -5.987654321098, 3.141592653589, 1.23456789);

  const std::array<double, 16> recovered = to_array16(to_affine(original));

  for (int i = 0; i < 16; ++i)
    REQUIRE(recovered[i] == original[i]);
}

// The float overload must preserve what it gets (float->double widens, but
// precision is already lost — this test documents the expected behaviour).
TEST_CASE("PosePrecision_FloatOverload_WidensLossily",
          "[pose][precision][float]") {
  const std::array<double, 16> original =
      make_pose(12.345678901234, -5.987654321098, 3.141592653589, 1.23456789);

  // Float round-trip should differ (documents the old behaviour for rendering
  // callers who knowingly accept float precision).
  Eigen::Affine3f aff_f;
  aff_f.matrix() = to_affine(original).matrix().cast<float>();
  const std::array<double, 16> via_float = to_array16(aff_f);
  bool any_different = false;
  for (int i = 0; i < 16; ++i)
    if (via_float[i] != original[i])
      any_different = true;
  REQUIRE(any_different); // float truncation is observable at double scale
}

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <io/rhino.hpp>
#include <reusex/types.hpp>

using Catch::Matchers::WithinAbs;

TEST_CASE("configure_rhino_model creates valid model", "[io][rhino]") {
  using namespace ReUseX;

  auto model = io::configure_rhino_model();

  REQUIRE(model != nullptr);
  REQUIRE(model->m_properties.m_Application.m_application_name.IsNotEmpty());

  // Verify unit system is set to meters
  REQUIRE(model->m_settings.m_ModelUnitsAndTolerances.m_unit_system == ON::LengthUnitSystem::Meters);

  // Verify tolerances are set
  REQUIRE(model->m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance > 0.0);
}

TEST_CASE("make_rhino_pointcloud converts PCL cloud", "[io][rhino]") {
  using namespace ReUseX;

  // Create minimal PCL cloud
  CloudPtr cloud(new Cloud);
  cloud->resize(3);
  cloud->points[0] = PointT{1.0f, 0.0f, 0.0f, 255, 0, 0};
  cloud->points[1] = PointT{0.0f, 1.0f, 0.0f, 0, 255, 0};
  cloud->points[2] = PointT{0.0f, 0.0f, 1.0f, 0, 0, 255};

  auto rhino_cloud = io::make_rhino_pointcloud(cloud);

  REQUIRE(rhino_cloud != nullptr);
  REQUIRE(rhino_cloud->m_P.Count() == 3);

  // Verify first point coordinates
  REQUIRE_THAT(rhino_cloud->m_P[0].x, WithinAbs(1.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[0].y, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[0].z, WithinAbs(0.0, 0.001));

  // Verify second point
  REQUIRE_THAT(rhino_cloud->m_P[1].x, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[1].y, WithinAbs(1.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[1].z, WithinAbs(0.0, 0.001));

  // Verify third point
  REQUIRE_THAT(rhino_cloud->m_P[2].x, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[2].y, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_P[2].z, WithinAbs(1.0, 0.001));

  // Verify colors are present
  REQUIRE(rhino_cloud->m_C.Count() == 3);
  REQUIRE(rhino_cloud->m_C[0].Red() == 255);
  REQUIRE(rhino_cloud->m_C[0].Green() == 0);
  REQUIRE(rhino_cloud->m_C[0].Blue() == 0);
}

TEST_CASE("make_rhino_pointcloud handles empty cloud", "[io][rhino]") {
  using namespace ReUseX;

  CloudPtr cloud(new Cloud);
  // Empty cloud

  auto rhino_cloud = io::make_rhino_pointcloud(cloud);

  REQUIRE(rhino_cloud != nullptr);
  REQUIRE(rhino_cloud->m_P.Count() == 0);
  REQUIRE(rhino_cloud->m_C.Count() == 0);
}

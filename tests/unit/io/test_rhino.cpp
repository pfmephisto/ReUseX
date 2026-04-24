// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <io/rhino.hpp>
#include <io/export_scene.hpp>
#include <reusex/types.hpp>

#include <pcl/common/io.h>

using Catch::Matchers::WithinAbs;

TEST_CASE("configure_rhino_model creates valid model", "[io][rhino]") {
  using namespace reusex;

  auto model = io::configure_rhino_model();

  REQUIRE(model != nullptr);
  REQUIRE(model->m_properties.m_Application.m_application_name.IsNotEmpty());

  // Verify unit system is set to meters
  REQUIRE(model->m_settings.m_ModelUnitsAndTolerances.m_unit_system == ON::LengthUnitSystem::Meters);

  // Verify tolerances are set
  REQUIRE(model->m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance > 0.0);
}

TEST_CASE("make_rhino_pointcloud converts PCL cloud", "[io][rhino]") {
  using namespace reusex;

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
  using namespace reusex;

  CloudPtr cloud(new Cloud);

  auto rhino_cloud = io::make_rhino_pointcloud(cloud);

  REQUIRE(rhino_cloud != nullptr);
  REQUIRE(rhino_cloud->m_P.Count() == 0);
  REQUIRE(rhino_cloud->m_C.Count() == 0);
}

TEST_CASE("make_rhino_pointcloud with normals uses actual normals", "[io][rhino]") {
  using namespace reusex;

  CloudPtr cloud(new Cloud);
  cloud->resize(2);
  cloud->points[0] = PointT{1.0f, 0.0f, 0.0f, 255, 0, 0};
  cloud->points[1] = PointT{0.0f, 1.0f, 0.0f, 0, 255, 0};

  CloudNPtr normals(new CloudN);
  normals->resize(2);
  normals->points[0].normal_x = 1.0f;
  normals->points[0].normal_y = 0.0f;
  normals->points[0].normal_z = 0.0f;
  normals->points[1].normal_x = 0.0f;
  normals->points[1].normal_y = 1.0f;
  normals->points[1].normal_z = 0.0f;

  auto rhino_cloud = io::make_rhino_pointcloud(cloud, normals);

  REQUIRE(rhino_cloud != nullptr);
  REQUIRE(rhino_cloud->m_N.Count() == 2);
  REQUIRE_THAT(rhino_cloud->m_N[0].x, WithinAbs(1.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_N[0].y, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(rhino_cloud->m_N[1].y, WithinAbs(1.0, 0.001));
}

TEST_CASE("make_rhino_mesh converts simple triangle mesh", "[io][rhino]") {
  using namespace reusex;

  pcl::PolygonMesh polygon_mesh;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize(3);
  cloud.points[0] = {0.0f, 0.0f, 0.0f};
  cloud.points[1] = {1.0f, 0.0f, 0.0f};
  cloud.points[2] = {0.0f, 1.0f, 0.0f};
  pcl::toPCLPointCloud2(cloud, polygon_mesh.cloud);

  pcl::Vertices tri;
  tri.vertices = {0, 1, 2};
  polygon_mesh.polygons.push_back(tri);

  auto rhino_mesh = io::make_rhino_mesh(polygon_mesh);

  REQUIRE(rhino_mesh != nullptr);
  REQUIRE(rhino_mesh->VertexCount() == 3);
  REQUIRE(rhino_mesh->FaceCount() == 1);
}

TEST_CASE("make_sphere_mesh creates valid sphere", "[io][rhino]") {
  using namespace reusex;

  auto sphere = io::make_sphere_mesh(0.0, 0.0, 0.0, 1.0, 4);

  REQUIRE(sphere != nullptr);
  REQUIRE(sphere->VertexCount() > 0);
  REQUIRE(sphere->FaceCount() > 0);

  // Top pole should be at (0,0,1)
  REQUIRE_THAT(sphere->m_V[0].x, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(sphere->m_V[0].y, WithinAbs(0.0, 0.001));
  REQUIRE_THAT(sphere->m_V[0].z, WithinAbs(1.0, 0.001));
}

TEST_CASE("export_to_rhino with cloud-only scene", "[io][rhino]") {
  using namespace reusex;

  io::ExportScene scene;

  CloudPtr cloud(new Cloud);
  cloud->resize(5);
  for (int i = 0; i < 5; ++i)
    cloud->points[i] = PointT{(float)i, 0.0f, 0.0f, 128, 128, 128};

  scene.cloud = io::ExportScene::CloudLayer{cloud, nullptr};

  auto model = io::export_to_rhino(scene);
  REQUIRE(model != nullptr);
}

TEST_CASE("export_to_rhino with empty scene produces model", "[io][rhino]") {
  using namespace reusex;

  io::ExportScene scene;
  auto model = io::export_to_rhino(scene);
  REQUIRE(model != nullptr);
}

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <reusex/types.hpp>

using namespace reusex;
using Catch::Approx;

TEST_CASE("Point cloud creation and basic operations", "[pointcloud]") {
  CloudPtr cloud(new Cloud());

  SECTION("Empty cloud initialization") {
    REQUIRE(cloud != nullptr);
    REQUIRE(cloud->empty());
    REQUIRE(cloud->size() == 0);
  }

  SECTION("Adding points to cloud") {
    PointT point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.r = 255;
    point.g = 0;
    point.b = 0;

    cloud->push_back(point);

    REQUIRE_FALSE(cloud->empty());
    REQUIRE(cloud->size() == 1);
    REQUIRE(cloud->points[0].x == Approx(1.0f));
    REQUIRE(cloud->points[0].y == Approx(2.0f));
    REQUIRE(cloud->points[0].z == Approx(3.0f));
    REQUIRE(cloud->points[0].r == 255);
  }

  SECTION("Cloud with multiple points") {
    cloud->resize(10);
    REQUIRE(cloud->size() == 10);

    for (size_t i = 0; i < cloud->size(); ++i) {
      cloud->points[i].x = static_cast<float>(i);
      cloud->points[i].y = static_cast<float>(i * 2);
      cloud->points[i].z = static_cast<float>(i * 3);
    }

    REQUIRE(cloud->points[5].x == Approx(5.0f));
    REQUIRE(cloud->points[5].y == Approx(10.0f));
    REQUIRE(cloud->points[5].z == Approx(15.0f));
  }
}

TEST_CASE("Normal cloud operations", "[normals]") {
  CloudNPtr normals(new CloudN());

  SECTION("Empty normals cloud") {
    REQUIRE(normals != nullptr);
    REQUIRE(normals->empty());
  }

  SECTION("Adding normals") {
    NormalT normal;
    normal.normal_x = 0.0f;
    normal.normal_y = 0.0f;
    normal.normal_z = 1.0f;

    normals->push_back(normal);

    REQUIRE(normals->size() == 1);
    REQUIRE(normals->points[0].normal_z == Approx(1.0f));
  }
}

TEST_CASE("Indices operations", "[indices]") {
  IndicesPtr indices(new Indices());

  SECTION("Empty indices") {
    REQUIRE(indices != nullptr);
    REQUIRE(indices->empty());
  }

  SECTION("Adding indices") {
    indices->push_back(0);
    indices->push_back(1);
    indices->push_back(2);

    REQUIRE(indices->size() == 3);
    REQUIRE((*indices)[0] == 0);
    REQUIRE((*indices)[1] == 1);
    REQUIRE((*indices)[2] == 2);
  }
}

TEST_CASE("Eigen vector container", "[eigen]") {
  EigenVectorContainer<double, 3> vectors;

  SECTION("Empty container") {
    REQUIRE(vectors.empty());
    REQUIRE(vectors.size() == 0);
  }

  SECTION("Adding Eigen vectors") {
    Eigen::Vector3d vec(1.0, 2.0, 3.0);
    vectors.push_back(vec);

    REQUIRE(vectors.size() == 1);
    REQUIRE(vectors[0](0) == Approx(1.0));
    REQUIRE(vectors[0](1) == Approx(2.0));
    REQUIRE(vectors[0](2) == Approx(3.0));
  }
}

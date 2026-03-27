// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/project_db.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/common/generate.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdio>
#include <filesystem>

using namespace ReUseX;
using namespace Catch::Matchers;

namespace fs = std::filesystem;

// Helper: create a temp database path that auto-cleans
struct TempDB {
  fs::path path;
  TempDB() : path(fs::temp_directory_path() /
                   ("test_projectdb_" +
                    std::to_string(reinterpret_cast<uintptr_t>(this)) +
                    ".rux")) {}
  ~TempDB() { fs::remove(path); }
};

// Helper: create a small XYZRGB cloud with known values
static CloudPtr makeXYZRGBCloud(size_t n) {
  auto cloud = std::make_shared<Cloud>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &pt = cloud->points[i];
    pt.x = static_cast<float>(i) * 0.1f;
    pt.y = static_cast<float>(i) * 0.2f;
    pt.z = static_cast<float>(i) * 0.3f;
    pt.r = static_cast<uint8_t>(i % 256);
    pt.g = static_cast<uint8_t>((i * 2) % 256);
    pt.b = static_cast<uint8_t>((i * 3) % 256);
    pt.a = 255;
  }
  return cloud;
}

static CloudNPtr makeNormalCloud(size_t n) {
  auto cloud = std::make_shared<CloudN>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &pt = cloud->points[i];
    pt.normal_x = static_cast<float>(i) * 0.01f;
    pt.normal_y = static_cast<float>(i) * 0.02f;
    pt.normal_z = static_cast<float>(i) * 0.03f;
    pt.curvature = static_cast<float>(i) * 0.001f;
  }
  return cloud;
}

static CloudLPtr makeLabelCloud(size_t n) {
  auto cloud = std::make_shared<CloudL>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    cloud->points[i].label = static_cast<uint32_t>(i % 10);
  }
  return cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr makeXYZCloud(size_t n) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &pt = cloud->points[i];
    pt.x = static_cast<float>(i) * 1.1f;
    pt.y = static_cast<float>(i) * 2.2f;
    pt.z = static_cast<float>(i) * 3.3f;
  }
  return cloud;
}

TEST_CASE("ProjectDB schema version on fresh DB", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.getSchemaVersion() == 2);
}

TEST_CASE("ProjectDB point cloud XYZRGB round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = makeXYZRGBCloud(100);
  db.savePointCloud("cloud", *original, "import");

  REQUIRE(db.hasPointCloud("cloud"));

  auto loaded = db.getPointCloudXYZRGB("cloud");
  REQUIRE(loaded->size() == original->size());
  REQUIRE(loaded->width == original->width);
  REQUIRE(loaded->height == original->height);

  for (size_t i = 0; i < original->size(); ++i) {
    REQUIRE(loaded->points[i].x == original->points[i].x);
    REQUIRE(loaded->points[i].y == original->points[i].y);
    REQUIRE(loaded->points[i].z == original->points[i].z);
    REQUIRE(loaded->points[i].rgba == original->points[i].rgba);
  }
}

TEST_CASE("ProjectDB point cloud Normal round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = makeNormalCloud(50);
  db.savePointCloud("normals", *original, "import");

  auto loaded = db.getPointCloudNormal("normals");
  REQUIRE(loaded->size() == original->size());

  for (size_t i = 0; i < original->size(); ++i) {
    REQUIRE(loaded->points[i].normal_x == original->points[i].normal_x);
    REQUIRE(loaded->points[i].normal_y == original->points[i].normal_y);
    REQUIRE(loaded->points[i].normal_z == original->points[i].normal_z);
    REQUIRE(loaded->points[i].curvature == original->points[i].curvature);
  }
}

TEST_CASE("ProjectDB point cloud Label round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = makeLabelCloud(200);
  db.savePointCloud("planes", *original, "segment_planes");

  auto loaded = db.getPointCloudLabel("planes");
  REQUIRE(loaded->size() == original->size());

  for (size_t i = 0; i < original->size(); ++i) {
    REQUIRE(loaded->points[i].label == original->points[i].label);
  }
}

TEST_CASE("ProjectDB point cloud PointXYZ round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = makeXYZCloud(30);
  db.savePointCloud("plane_centroids", *original, "segment_planes");

  auto loaded = db.getPointCloudXYZ("plane_centroids");
  REQUIRE(loaded->size() == original->size());

  for (size_t i = 0; i < original->size(); ++i) {
    REQUIRE(loaded->points[i].x == original->points[i].x);
    REQUIRE(loaded->points[i].y == original->points[i].y);
    REQUIRE(loaded->points[i].z == original->points[i].z);
  }
}

TEST_CASE("ProjectDB hasPointCloud and deletePointCloud", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_FALSE(db.hasPointCloud("nonexistent"));

  auto cloud = makeXYZRGBCloud(10);
  db.savePointCloud("test_cloud", *cloud);

  REQUIRE(db.hasPointCloud("test_cloud"));

  db.deletePointCloud("test_cloud");
  REQUIRE_FALSE(db.hasPointCloud("test_cloud"));
}

TEST_CASE("ProjectDB listPointClouds", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE(db.listPointClouds().empty());

  db.savePointCloud("cloud", *makeXYZRGBCloud(5));
  db.savePointCloud("normals", *makeNormalCloud(5));
  db.savePointCloud("labels", *makeLabelCloud(5));

  auto names = db.listPointClouds();
  REQUIRE(names.size() == 3);
  REQUIRE(names[0] == "cloud");
  REQUIRE(names[1] == "normals");
  REQUIRE(names[2] == "labels");
}

TEST_CASE("ProjectDB UPSERT replaces existing cloud", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto cloud1 = makeXYZRGBCloud(10);
  db.savePointCloud("cloud", *cloud1);

  auto cloud2 = makeXYZRGBCloud(20);
  db.savePointCloud("cloud", *cloud2, "reimport");

  auto loaded = db.getPointCloudXYZRGB("cloud");
  REQUIRE(loaded->size() == 20);

  // Verify only one entry in list
  REQUIRE(db.listPointClouds().size() == 1);
}

TEST_CASE("ProjectDB label definitions save/load", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto labels = makeLabelCloud(10);
  db.savePointCloud("labels", *labels, "annotate");

  std::map<int, std::string> labelMap = {
      {0, "wall"}, {1, "floor"}, {2, "ceiling"}, {3, "window"}, {4, "door"}};
  db.saveLabelDefinitions("labels", labelMap);

  auto loaded = db.getLabelDefinitions("labels");
  REQUIRE(loaded.size() == labelMap.size());
  REQUIRE(loaded[0] == "wall");
  REQUIRE(loaded[1] == "floor");
  REQUIRE(loaded[2] == "ceiling");
  REQUIRE(loaded[3] == "window");
  REQUIRE(loaded[4] == "door");
}

TEST_CASE("ProjectDB mesh save/load round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // Create a simple triangle mesh
  pcl::PolygonMesh mesh;
  Cloud meshCloud;
  meshCloud.width = 3;
  meshCloud.height = 1;
  meshCloud.is_dense = true;
  meshCloud.points.resize(3);
  meshCloud.points[0] = pcl::PointXYZRGB(0.0f, 0.0f, 0.0f);
  meshCloud.points[1] = pcl::PointXYZRGB(1.0f, 0.0f, 0.0f);
  meshCloud.points[2] = pcl::PointXYZRGB(0.0f, 1.0f, 0.0f);
  pcl::toPCLPointCloud2(meshCloud, mesh.cloud);

  pcl::Vertices tri;
  tri.vertices = {0, 1, 2};
  mesh.polygons.push_back(tri);

  REQUIRE_FALSE(db.hasMesh("test_mesh"));
  db.saveMesh("test_mesh", mesh, "mesh_gen");
  REQUIRE(db.hasMesh("test_mesh"));

  auto loaded = db.getMesh("test_mesh");
  REQUIRE(loaded != nullptr);
  REQUIRE(loaded->polygons.size() == 1);
  REQUIRE(loaded->polygons[0].vertices.size() == 3);
}

TEST_CASE("ProjectDB pipeline log start/end", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  int logId = db.logPipelineStart("import", R"({"resolution": 0.05})");
  REQUIRE(logId > 0);

  // End with success
  REQUIRE_NOTHROW(db.logPipelineEnd(logId, true));

  // Start another and end with failure
  int logId2 = db.logPipelineStart("segment_planes");
  REQUIRE_NOTHROW(db.logPipelineEnd(logId2, false, "Out of memory"));
}

TEST_CASE("ProjectDB type mismatch throws on load", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.savePointCloud("cloud", *makeXYZRGBCloud(5));

  REQUIRE_THROWS_AS(db.getPointCloudNormal("cloud"), std::runtime_error);
  REQUIRE_THROWS_AS(db.getPointCloudLabel("cloud"), std::runtime_error);
  REQUIRE_THROWS_AS(db.getPointCloudXYZ("cloud"), std::runtime_error);
}

TEST_CASE("ProjectDB nonexistent cloud throws on load", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_THROWS_AS(db.getPointCloudXYZRGB("missing"), std::runtime_error);
}

TEST_CASE("ProjectDB fresh DB includes all v1 tables", "[projectdb]") {
  TempDB tmp;

  // Fresh DB should create passport tables + v1 tables in one pass
  ProjectDB db(tmp.path);
  REQUIRE(db.getSchemaVersion() == 2);

  // V1 tables should work
  REQUIRE_FALSE(db.hasPointCloud("anything"));
  REQUIRE(db.listPointClouds().empty());
  REQUIRE_FALSE(db.hasMesh("anything"));

  // Passport tables should also work (validateSchema checks them)
  REQUIRE_NOTHROW(db.validateSchema());
}

TEST_CASE("ProjectDB read-only mode", "[projectdb]") {
  TempDB tmp;

  // Create DB first in write mode
  { ProjectDB db(tmp.path); }

  // Open in read-only mode
  ProjectDB rodb(tmp.path, true);
  REQUIRE(rodb.isOpen());
  REQUIRE(rodb.listPointClouds().empty());
}

// ── Sensor Frame Tests ──────────────────────────────────────────────

TEST_CASE("ProjectDB sensor frame save/load round-trip", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // Create a small color image
  cv::Mat original(480, 640, CV_8UC3);
  cv::randu(original, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

  db.saveSensorFrame(1, original);

  cv::Mat loaded = db.getSensorFrameImage(1);
  REQUIRE_FALSE(loaded.empty());
  REQUIRE(loaded.rows == original.rows);
  REQUIRE(loaded.cols == original.cols);
  REQUIRE(loaded.channels() == original.channels());
}

TEST_CASE("ProjectDB getSensorFrameIds returns correct list", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  cv::Mat img(100, 100, CV_8UC3, cv::Scalar(128, 128, 128));

  db.saveSensorFrame(10, img);
  db.saveSensorFrame(5, img);
  db.saveSensorFrame(20, img);

  auto ids = db.getSensorFrameIds();
  REQUIRE(ids.size() == 3);
  // Should be sorted by node_id
  REQUIRE(ids[0] == 5);
  REQUIRE(ids[1] == 10);
  REQUIRE(ids[2] == 20);
}

TEST_CASE("ProjectDB getSensorFrameImage returns empty for missing node",
          "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  cv::Mat result = db.getSensorFrameImage(999);
  REQUIRE(result.empty());
}

// ── Segmentation Image Tests ────────────────────────────────────────

TEST_CASE("ProjectDB segmentation image save/load round-trip",
          "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // Create a sensor frame first (foreign key)
  cv::Mat color(100, 100, CV_8UC3, cv::Scalar(128, 128, 128));
  db.saveSensorFrame(1, color);

  // Create labels: -1 for background, 0-4 for classes
  cv::Mat labels(100, 100, CV_32S);
  for (int r = 0; r < 100; ++r) {
    for (int c = 0; c < 100; ++c) {
      if (r < 20)
        labels.at<int>(r, c) = -1; // background
      else
        labels.at<int>(r, c) = (r + c) % 5; // class labels 0-4
    }
  }

  db.saveSegmentationImage(1, labels);

  cv::Mat loaded = db.getSegmentationImage(1);
  REQUIRE_FALSE(loaded.empty());
  REQUIRE(loaded.rows == labels.rows);
  REQUIRE(loaded.cols == labels.cols);
  REQUIRE(loaded.type() == CV_32S);

  // Exact value preservation
  for (int r = 0; r < 100; ++r) {
    for (int c = 0; c < 100; ++c) {
      REQUIRE(loaded.at<int>(r, c) == labels.at<int>(r, c));
    }
  }
}

TEST_CASE("ProjectDB hasSegmentationImage before/after save",
          "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  cv::Mat color(50, 50, CV_8UC3, cv::Scalar(0, 0, 0));
  db.saveSensorFrame(42, color);

  REQUIRE_FALSE(db.hasSegmentationImage(42));

  cv::Mat labels(50, 50, CV_32S, cv::Scalar(0));
  db.saveSegmentationImage(42, labels);

  REQUIRE(db.hasSegmentationImage(42));
}

TEST_CASE("ProjectDB saveSegmentationImages batch save", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  cv::Mat color(50, 50, CV_8UC3, cv::Scalar(0, 0, 0));
  db.saveSensorFrame(1, color);
  db.saveSensorFrame(2, color);
  db.saveSensorFrame(3, color);

  std::vector<int> nodeIds = {1, 2, 3};
  std::vector<cv::Mat> labels;
  for (int i = 0; i < 3; ++i) {
    labels.emplace_back(50, 50, CV_32S, cv::Scalar(i));
  }

  db.saveSegmentationImages(nodeIds, labels);

  for (int i = 0; i < 3; ++i) {
    REQUIRE(db.hasSegmentationImage(nodeIds[i]));
    cv::Mat loaded = db.getSegmentationImage(nodeIds[i]);
    REQUIRE(loaded.at<int>(0, 0) == i);
  }
}

TEST_CASE("ProjectDB label encoding: background -1 preserved", "[projectdb]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  cv::Mat color(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));
  db.saveSensorFrame(1, color);

  // All background
  cv::Mat labels(10, 10, CV_32S, cv::Scalar(-1));
  db.saveSegmentationImage(1, labels);

  cv::Mat loaded = db.getSegmentationImage(1);
  for (int r = 0; r < 10; ++r) {
    for (int c = 0; c < 10; ++c) {
      REQUIRE(loaded.at<int>(r, c) == -1);
    }
  }
}

TEST_CASE("ProjectDB fresh DB has schema version 2", "[projectdb]") {
  TempDB tmp;

  ProjectDB db(tmp.path);
  REQUIRE(db.getSchemaVersion() == 2);

  // V2 tables should work
  REQUIRE_FALSE(db.hasSegmentationImage(1));
  REQUIRE(db.getSensorFrameIds().empty());
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// End-to-end test of the `rux create attributes` stage core, driven by a fake
// (network-free) IVlmClient injected via describe_with_client() (#373). The
// stage iterates MATERIAL passports, resolves each to its linked instance(s),
// crops their best view and stores a material-keyed annotation.

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <types.hpp>
#include <vision/IVlmClient.hpp>
#include <vision/describe.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;
using reusex::core::SensorIntrinsics;
using reusex::vision::DescribeConfig;
using reusex::vision::IVlmClient;
using reusex::vision::VlmResult;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_describe_client") {}
};

// A fake client: records how many times it was called and returns a canned,
// parseable result. Never touches the network.
struct FakeClient : IVlmClient {
  int calls = 0;
  bool return_ok = true;
  VlmResult describe(const cv::Mat &crop, const std::string &prompt) override {
    (void)prompt;
    ++calls;
    REQUIRE_FALSE(crop.empty()); // stage must hand us a real crop
    VlmResult r;
    r.ok = return_ok;
    if (return_ok) {
      r.description = "a wooden chair";
      r.attributes = {{"material", "wood"}, {"condition", "good"}};
    }
    r.raw_json = R"({"description":"a wooden chair"})";
    return r;
  }
};

SensorIntrinsics makeIntrinsics() {
  SensorIntrinsics k;
  k.fx = 500;
  k.fy = 500;
  k.cx = 320;
  k.cy = 240;
  k.width = 640;
  k.height = 480;
  return k;
}

void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

// Build a project with a fused cloud + an index-aligned instance-label cloud +
// one posed sensor frame whose view contains the instance points. Instance 1 is
// linked to material "mat-1".
void seedProject(ProjectDB &db) {
  auto xyz = std::make_shared<Cloud>();
  auto labels = std::make_shared<CloudL>();
  const int n = 60;
  xyz->points.resize(n);
  labels->points.resize(n);
  for (int i = 0; i < n; ++i) {
    // small cluster around (0,0,2)
    xyz->points[i].x = 0.05f * ((i % 5) - 2);
    xyz->points[i].y = 0.05f * ((i % 3) - 1);
    xyz->points[i].z = 2.0f;
    xyz->points[i].r = 200;
    xyz->points[i].g = 100;
    xyz->points[i].b = 50;
    labels->points[i].label = 1;
  }
  xyz->width = n;
  xyz->height = 1;
  xyz->is_dense = false;
  labels->width = n;
  labels->height = 1;
  labels->is_dense = false;

  db.save_point_cloud("cloud", *xyz, "clouds");
  db.save_point_cloud("instances", *labels, "segment_instances");

  std::vector<ProjectDB::InstanceRecord> recs = {{1u, "guid-1", 1, n}};
  db.save_instances("instances", recs);

  makePassport(db, "mat-1");
  db.set_instance_material("instances", 1, "mat-1");

  // Identity-posed frame with a color image and matching intrinsics.
  cv::Mat color(480, 640, CV_8UC3, cv::Scalar(120, 120, 120));
  cv::Mat depth, conf;
  std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  db.save_sensor_frame(0, color, depth, conf, pose, makeIntrinsics(), 0.0);
}

} // namespace

TEST_CASE("DescribeWithClient_OneMaterialWithView_StoresAnnotation",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    seedProject(db);
  }

  FakeClient client;
  DescribeConfig cfg;
  cfg.min_view_points = 5;
  int stored = reusex::vision::describe_with_client(tmp.path, cfg, client);

  CHECK(stored == 1);
  CHECK(client.calls == 1);

  ProjectDB db(tmp.path);
  auto ann = db.material_annotation("mat-1");
  REQUIRE(ann.has_value());
  CHECK(ann->description == "a wooden chair");
  REQUIRE(ann->attributes.size() == 2);
  bool has_material = false;
  for (const auto &[k, v] : ann->attributes)
    if (k == "material" && v == "wood")
      has_material = true;
  CHECK(has_material);
  CHECK(ann->provider_model.find("qwen2.5vl") != std::string::npos);
}

TEST_CASE("DescribeWithClient_MaterialWithNoLinkedInstance_SkipsIt",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    seedProject(db);
    // A second material with NO instance link: it must be skipped, and the
    // model is never asked for it.
    makePassport(db, "mat-unlinked");
  }

  FakeClient client;
  DescribeConfig cfg;
  cfg.min_view_points = 5;
  int stored = reusex::vision::describe_with_client(tmp.path, cfg, client);

  CHECK(stored == 1);       // only mat-1
  CHECK(client.calls == 1); // never called for the unlinked material

  ProjectDB db(tmp.path);
  CHECK(db.material_annotation("mat-1").has_value());
  CHECK_FALSE(db.material_annotation("mat-unlinked").has_value());
}

TEST_CASE("DescribeWithClient_ModelReturnsNotOk_LeavesAnnotationUnset",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    seedProject(db);
  }

  FakeClient client;
  client.return_ok = false; // model gave nothing usable
  DescribeConfig cfg;
  cfg.min_view_points = 5;
  int stored = reusex::vision::describe_with_client(tmp.path, cfg, client);

  CHECK(stored == 0);
  CHECK(client.calls == 1); // it was asked

  ProjectDB db(tmp.path);
  // No fabrication: nothing stored (STANDARDS §5).
  CHECK_FALSE(db.material_annotation("mat-1").has_value());
}

TEST_CASE("DescribeWithClient_SkipExisting_DoesNotRecallModel",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    seedProject(db);
  }

  DescribeConfig cfg;
  cfg.min_view_points = 5;

  FakeClient first;
  REQUIRE(reusex::vision::describe_with_client(tmp.path, cfg, first) == 1);

  cfg.skip_existing = true;
  FakeClient second;
  int stored = reusex::vision::describe_with_client(tmp.path, cfg, second);
  CHECK(stored == 0);
  CHECK(second.calls == 0); // skipped without asking the model again
}

TEST_CASE("DescribeWithClient_MissingInstancesCloud_Throws",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path); // empty project (creates schema, no instances)
  }

  FakeClient client;
  DescribeConfig cfg;
  REQUIRE_THROWS_AS(reusex::vision::describe_with_client(tmp.path, cfg, client),
                    std::runtime_error);
}

TEST_CASE("DescribeWithClient_NoMaterials_ReturnsZeroWithoutCalling",
          "[vision][attributes]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    // Seed cloud + instances + a frame but NO material passports.
    auto xyz = std::make_shared<Cloud>();
    auto labels = std::make_shared<CloudL>();
    const int n = 10;
    xyz->points.resize(n);
    labels->points.resize(n);
    for (int i = 0; i < n; ++i) {
      xyz->points[i].z = 2.0f;
      labels->points[i].label = 1;
    }
    xyz->width = n;
    xyz->height = 1;
    labels->width = n;
    labels->height = 1;
    db.save_point_cloud("cloud", *xyz, "clouds");
    db.save_point_cloud("instances", *labels, "segment_instances");
    std::vector<ProjectDB::InstanceRecord> recs = {{1u, "guid-1", 1, n}};
    db.save_instances("instances", recs);
    cv::Mat color(480, 640, CV_8UC3, cv::Scalar(120, 120, 120));
    cv::Mat depth, conf;
    std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1};
    db.save_sensor_frame(0, color, depth, conf, pose, makeIntrinsics(), 0.0);
  }

  FakeClient client;
  DescribeConfig cfg;
  cfg.min_view_points = 5;
  int stored = reusex::vision::describe_with_client(tmp.path, cfg, client);
  CHECK(stored == 0);
  CHECK(client.calls == 0); // nothing to describe
}

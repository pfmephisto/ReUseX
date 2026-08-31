// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/validate.hpp>
#include <types.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;
using reusex::core::ValidationSeverity;
namespace fs = std::filesystem;

namespace {

struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_validate_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempDB() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
  }
};

CloudLPtr makeLabelCloud(size_t n) {
  auto c = std::make_shared<CloudL>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  for (size_t i = 0; i < n; ++i)
    c->points[i].label = static_cast<uint32_t>((i % 2) + 1);
  return c;
}

CloudPtr makeRgbCloud(size_t n) {
  auto c = std::make_shared<Cloud>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

CloudNPtr makeNormalCloud(size_t n) {
  auto c = std::make_shared<CloudN>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> makeXyzCloud(size_t n) {
  auto c = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

// Populate a project with the per-point and per-plane clouds the mesh stage
// consumes, all consistently sized so the stage contract is satisfied.
void seedMeshInputs(ProjectDB &db, size_t points, size_t planes) {
  db.save_point_cloud("cloud", *makeRgbCloud(points), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(points), "create_clouds");
  db.save_point_cloud("planes", *makeLabelCloud(points), "segment_planes");
  db.save_point_cloud("rooms", *makeLabelCloud(points), "segment_rooms");
  db.save_point_cloud("plane_centroids", *makeXyzCloud(planes),
                      "segment_planes");
  db.save_point_cloud("plane_normals", *makeNormalCloud(planes),
                      "segment_planes");
}

void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

bool hasCheck(const std::vector<reusex::core::ValidationIssue> &issues,
              const std::string &check) {
  return std::any_of(issues.begin(), issues.end(),
                     [&](const auto &i) { return i.check == check; });
}

} // namespace

TEST_CASE("validate: clean project reports no issues", "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeLabelCloud(4), "segment_instances");
  std::vector<ProjectDB::InstanceRecord> recs = {{1, "g1", 1, 2},
                                                 {2, "g2", 1, 2}};
  db.save_instances("instances", recs);
  db.save_label_definitions("instances",
                            {{1, "SM1-1 (2p)"}, {2, "SM1-2 (2p)"}});
  makePassport(db, "mat-1");
  db.set_instance_material("instances", 1, "mat-1");

  auto report = reusex::core::validate_project(db);
  // Passport mat-1 is linked; g? passports linked. No errors expected.
  REQUIRE(report.error_count() == 0);
  REQUIRE(report.ok());
}

TEST_CASE("validate: orphaned passport flagged as warning",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  makePassport(db, "lonely-passport");

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_orphaned_passports(db, issues);
  REQUIRE(hasCheck(issues, "orphaned_passport"));
  REQUIRE(std::all_of(issues.begin(), issues.end(), [](const auto &i) {
    return i.severity == ValidationSeverity::warning;
  }));
  // Warnings do not make the project invalid.
  auto report = reusex::core::validate_project(db);
  REQUIRE(report.ok());
}

TEST_CASE("validate: instance without label def is an error",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeLabelCloud(4), "segment_instances");
  std::vector<ProjectDB::InstanceRecord> recs = {{1, "g1", 1, 2},
                                                 {2, "g2", 1, 2}};
  db.save_instances("instances", recs);
  // Only define label 1; instance 2 has no definition.
  db.save_label_definitions("instances", {{1, "SM1-1 (2p)"}});

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_instances_without_label_defs(db, issues);
  REQUIRE(hasCheck(issues, "instance_without_label_def"));

  auto report = reusex::core::validate_project(db);
  REQUIRE_FALSE(report.ok());
}

TEST_CASE("validate: sibling clouds of different sizes are an error",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(10), "create_clouds");
  db.save_point_cloud("labels", *makeLabelCloud(7), "annotate"); // mismatch

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_sibling_cloud_sizes(db, issues);
  REQUIRE(hasCheck(issues, "sibling_size_mismatch"));

  auto report = reusex::core::validate_project(db);
  REQUIRE_FALSE(report.ok());
}

TEST_CASE("validate: matching sibling clouds pass size check",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(8), "create_clouds");
  db.save_point_cloud("labels", *makeLabelCloud(8), "annotate");

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_sibling_cloud_sizes(db, issues);
  REQUIRE_FALSE(hasCheck(issues, "sibling_size_mismatch"));
}

// ── Stage input contracts (#222) ───────────────────────────────────────────

using reusex::core::PipelineStage;

TEST_CASE("validate --stage: stage name parsing round-trips",
          "[core][validate][stage]") {
  using reusex::core::parse_pipeline_stage;
  using reusex::core::to_string;
  REQUIRE(parse_pipeline_stage("mesh") == PipelineStage::mesh);
  REQUIRE(parse_pipeline_stage("planes") == PipelineStage::planes);
  // "register" is an alias for "optimize".
  REQUIRE(parse_pipeline_stage("register") == PipelineStage::optimize);
  REQUIRE(parse_pipeline_stage("optimize") == PipelineStage::optimize);
  REQUIRE_FALSE(parse_pipeline_stage("bogus").has_value());
  REQUIRE(to_string(PipelineStage::mesh) == "mesh");
}

TEST_CASE("validate --stage planes: missing inputs are errors",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path); // empty project

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("validate --stage planes: present + aligned inputs pass",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(16), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(16), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE(report.ok());
  REQUIRE(report.error_count() == 0);
}

TEST_CASE("validate --stage planes: misaligned inputs are an error",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(16), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(15), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "stage_input_size_mismatch"));
}

TEST_CASE("validate --stage mesh: full input set passes",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, /*points=*/32, /*planes=*/5);

  auto report = reusex::core::validate_stage(db, PipelineStage::mesh);
  REQUIRE(report.ok());
  REQUIRE(report.error_count() == 0);
}

TEST_CASE("validate --stage mesh: missing rooms flagged",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, 32, 5);
  // Remove one required sibling to break the contract.
  db.delete_point_cloud("rooms");

  auto report = reusex::core::validate_stage(db, PipelineStage::mesh);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("validate --stage import: no in-project prerequisites",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path); // empty project
  auto report = reusex::core::validate_stage(db, PipelineStage::import);
  REQUIRE(report.ok());
}

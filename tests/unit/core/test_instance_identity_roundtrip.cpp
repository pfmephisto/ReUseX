// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end GUID stability across an instance-cloud regeneration, exercising
// the reconcile algorithm together with the ProjectDB instances table and
// FK-enforced material links (issues #207, #208).

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/guid.hpp>
#include <geometry/reconcile_instances.hpp>
#include <types.hpp>

#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;
using namespace reusex::geometry;
namespace fs = std::filesystem;

namespace {

struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_identity_rt_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempDB() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
  }
};

CloudLPtr makeLabels(const std::vector<uint32_t> &labels) {
  auto c = std::make_shared<CloudL>();
  c->width = static_cast<uint32_t>(labels.size());
  c->height = 1;
  c->is_dense = false;
  c->points.resize(labels.size());
  for (size_t i = 0; i < labels.size(); ++i)
    c->points[i].label = labels[i];
  return c;
}

void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

// Simulate the CLI create-instances flow: reconcile against prior state, save
// the new cloud + instances rows, and carry material links to matched
// instances. Returns the reconcile result for inspection.
ReconcileResult regenerate(ProjectDB &db, const std::string &cloud,
                           const CloudLPtr &new_labels,
                           const std::map<uint32_t, uint32_t> &new_sem,
                           const std::map<uint32_t, size_t> &new_sizes) {
  CloudLPtr old_labels;
  std::vector<PriorInstance> prior;
  std::map<int, std::string> old_links;
  if (db.has_point_cloud(cloud)) {
    old_labels = db.point_cloud_label(cloud);
    for (const auto &oi : db.instances(cloud))
      prior.push_back({oi.instance_id, oi.semantic_class, oi.guid});
    old_links = db.instance_materials(cloud);
  }

  auto r = reconcile_instance_identities(
      old_labels.get(), *new_labels, prior, new_sem, new_sizes,
      []() { return reusex::core::generate_guid(); });

  db.save_point_cloud(cloud, *new_labels, "segment_instances");

  std::vector<ProjectDB::InstanceRecord> recs;
  std::map<uint32_t, std::string> defs_map;
  for (const auto &ri : r.instances) {
    recs.push_back(
        {ri.instance_id, ri.guid, ri.semantic_class, ri.point_count});
  }
  db.save_instances(cloud, recs);

  std::map<int, std::string> defs;
  for (const auto &[id, sem] : new_sem)
    defs[static_cast<int>(id)] =
        "SM" + std::to_string(sem) + "-" + std::to_string(id) + " (0p)";
  db.save_label_definitions(cloud, defs);

  // Carry links to matched instances.
  std::map<uint32_t, uint32_t> old_to_new;
  for (const auto &ri : r.instances)
    if (ri.carried_over)
      old_to_new[ri.matched_old_id] = ri.instance_id;
  for (const auto &[old_id, guid] : old_links) {
    auto it = old_to_new.find(static_cast<uint32_t>(old_id));
    if (it != old_to_new.end())
      db.set_instance_material(cloud, static_cast<int>(it->second), guid);
  }
  return r;
}

} // namespace

TEST_CASE("GUID + material link survive an unchanged regeneration",
          "[projectdb][instances][identity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const std::string cloud = "instances";

  // First run: two instances of class 3.
  auto labels = makeLabels({1, 1, 1, 2, 2, 2});
  std::map<uint32_t, uint32_t> sem = {{1, 3}, {2, 3}};
  std::map<uint32_t, size_t> sizes = {{1, 3}, {2, 3}};
  regenerate(db, cloud, labels, sem, sizes);

  std::string guid1_before = db.instance_guid(cloud, 1);
  std::string guid2_before = db.instance_guid(cloud, 2);

  // Link a material to instance 1.
  makePassport(db, "mat-1");
  db.set_instance_material(cloud, 1, "mat-1");

  // Second run: identical clouds -> GUIDs and link must persist.
  auto labels2 = makeLabels({1, 1, 1, 2, 2, 2});
  auto r = regenerate(db, cloud, labels2, sem, sizes);

  REQUIRE(r.matched_count == 2);
  REQUIRE(r.fresh_count == 0);
  REQUIRE(db.instance_guid(cloud, 1) == guid1_before);
  REQUIRE(db.instance_guid(cloud, 2) == guid2_before);

  auto link = db.instance_material_guid(cloud, 1);
  REQUIRE(link.has_value());
  REQUIRE(*link == "mat-1");
}

TEST_CASE("Renumbered instances keep GUIDs and links follow the object",
          "[projectdb][instances][identity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const std::string cloud = "instances";

  auto labels = makeLabels({1, 1, 1, 2, 2, 2});
  std::map<uint32_t, uint32_t> sem = {{1, 3}, {2, 3}};
  std::map<uint32_t, size_t> sizes = {{1, 3}, {2, 3}};
  regenerate(db, cloud, labels, sem, sizes);

  std::string guid_of_first_object = db.instance_guid(cloud, 1);
  makePassport(db, "mat-obj");
  db.set_instance_material(cloud, 1, "mat-obj");

  // Second run: instance ids swapped spatially.
  auto labels2 = makeLabels({2, 2, 2, 1, 1, 1});
  auto r = regenerate(db, cloud, labels2, sem, sizes);
  REQUIRE(r.matched_count == 2);

  // The first object now has instance id 2; its GUID and link move with it.
  REQUIRE(db.instance_guid(cloud, 2) == guid_of_first_object);
  auto link = db.instance_material_guid(cloud, 2);
  REQUIRE(link.has_value());
  REQUIRE(*link == "mat-obj");
}

TEST_CASE("Vanished instance orphans its link but keeps the passport",
          "[projectdb][instances][identity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const std::string cloud = "instances";

  auto labels = makeLabels({1, 1, 1, 2, 2, 2});
  std::map<uint32_t, uint32_t> sem = {{1, 3}, {2, 3}};
  std::map<uint32_t, size_t> sizes = {{1, 3}, {2, 3}};
  regenerate(db, cloud, labels, sem, sizes);
  makePassport(db, "mat-2");
  db.set_instance_material(cloud, 2, "mat-2");

  // Second run: instance 2's points become unlabeled; only instance 1 remains.
  auto labels2 = makeLabels({1, 1, 1, 0, 0, 0});
  std::map<uint32_t, uint32_t> sem2 = {{1, 3}};
  std::map<uint32_t, size_t> sizes2 = {{1, 3}};
  auto r = regenerate(db, cloud, labels2, sem2, sizes2);

  REQUIRE(r.orphaned_old.size() == 1);
  REQUIRE(r.orphaned_old[0].instance_id == 2);

  // Link is gone (its instance no longer exists) but the passport remains.
  REQUIRE(db.instance_materials(cloud).count(2) == 0);
  auto passport = db.material_passport("mat-2");
  REQUIRE(passport.metadata.document_guid == "mat-2");
}

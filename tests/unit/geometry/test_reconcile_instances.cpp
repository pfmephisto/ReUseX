// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <geometry/reconcile_instances.hpp>
#include <types.hpp>

#include <map>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

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

// Deterministic guid factory for tests.
std::function<std::string()> counterGuid(int &n) {
  return [&n]() { return "fresh-" + std::to_string(++n); };
}

} // namespace

TEST_CASE("reconcile: identical clouds carry every GUID over",
          "[geometry][reconcile]") {
  // Two instances of the same semantic class, unchanged.
  // Points 0-2 = instance 1, points 3-5 = instance 2.
  auto old_l = makeLabels({1, 1, 1, 2, 2, 2});
  auto new_l = makeLabels({1, 1, 1, 2, 2, 2});

  std::vector<PriorInstance> prior = {{1, 7, "guid-old-1"},
                                      {2, 7, "guid-old-2"}};
  std::map<uint32_t, uint32_t> new_sem = {{1, 7}, {2, 7}};
  std::map<uint32_t, size_t> new_sizes = {{1, 3}, {2, 3}};

  int n = 0;
  auto r = reconcile_instance_identities(old_l.get(), *new_l, prior, new_sem,
                                         new_sizes, counterGuid(n));

  REQUIRE(r.matched_count == 2);
  REQUIRE(r.fresh_count == 0);
  REQUIRE(r.orphaned_old.empty());
  REQUIRE(r.instances.size() == 2);
  // GUIDs carried through, keyed by new instance id.
  std::map<uint32_t, std::string> byId;
  for (const auto &ri : r.instances)
    byId[ri.instance_id] = ri.guid;
  REQUIRE(byId.at(1) == "guid-old-1");
  REQUIRE(byId.at(2) == "guid-old-2");
}

TEST_CASE("reconcile: renumbered instances still match by overlap",
          "[geometry][reconcile]") {
  // Same spatial split but instance ids swapped between runs.
  auto old_l = makeLabels({1, 1, 1, 2, 2, 2});
  auto new_l = makeLabels({2, 2, 2, 1, 1, 1});

  std::vector<PriorInstance> prior = {{1, 7, "guid-A"}, {2, 7, "guid-B"}};
  std::map<uint32_t, uint32_t> new_sem = {{1, 7}, {2, 7}};
  std::map<uint32_t, size_t> new_sizes = {{1, 3}, {2, 3}};

  int n = 0;
  auto r = reconcile_instance_identities(old_l.get(), *new_l, prior, new_sem,
                                         new_sizes, counterGuid(n));

  REQUIRE(r.matched_count == 2);
  REQUIRE(r.fresh_count == 0);
  std::map<uint32_t, std::string> byId;
  for (const auto &ri : r.instances)
    byId[ri.instance_id] = ri.guid;
  // new instance 2 occupies old instance 1's points -> guid-A.
  REQUIRE(byId.at(2) == "guid-A");
  REQUIRE(byId.at(1) == "guid-B");
}

TEST_CASE("reconcile: different semantic class never matches",
          "[geometry][reconcile]") {
  auto old_l = makeLabels({1, 1, 1});
  auto new_l = makeLabels({1, 1, 1});

  std::vector<PriorInstance> prior = {{1, 5, "guid-old"}};
  std::map<uint32_t, uint32_t> new_sem = {{1, 9}}; // different class
  std::map<uint32_t, size_t> new_sizes = {{1, 3}};

  int n = 0;
  auto r = reconcile_instance_identities(old_l.get(), *new_l, prior, new_sem,
                                         new_sizes, counterGuid(n));

  REQUIRE(r.matched_count == 0);
  REQUIRE(r.fresh_count == 1);
  REQUIRE(r.orphaned_old.size() == 1);
  REQUIRE(r.instances[0].guid == "fresh-1");
  REQUIRE_FALSE(r.instances[0].carried_over);
}

TEST_CASE("reconcile: below-threshold overlap yields fresh guid + orphan",
          "[geometry][reconcile]") {
  // Old instance 1 covers points 0-3. New instance 1 covers only point 0
  // plus new points 4,5 -> overlap = 1/3 < 0.5.
  auto old_l = makeLabels({1, 1, 1, 1, 0, 0});
  auto new_l = makeLabels({1, 0, 0, 0, 1, 1});

  std::vector<PriorInstance> prior = {{1, 7, "guid-old"}};
  std::map<uint32_t, uint32_t> new_sem = {{1, 7}};
  std::map<uint32_t, size_t> new_sizes = {{1, 3}};

  int n = 0;
  auto r = reconcile_instance_identities(old_l.get(), *new_l, prior, new_sem,
                                         new_sizes, counterGuid(n));

  REQUIRE(r.matched_count == 0);
  REQUIRE(r.fresh_count == 1);
  REQUIRE(r.orphaned_old.size() == 1);
  REQUIRE(r.orphaned_old[0].guid == "guid-old");
}

TEST_CASE("reconcile: no prior state assigns all fresh guids",
          "[geometry][reconcile]") {
  auto new_l = makeLabels({1, 1, 2, 2});
  std::vector<PriorInstance> prior;
  std::map<uint32_t, uint32_t> new_sem = {{1, 3}, {2, 3}};
  std::map<uint32_t, size_t> new_sizes = {{1, 2}, {2, 2}};

  int n = 0;
  auto r = reconcile_instance_identities(nullptr, *new_l, prior, new_sem,
                                         new_sizes, counterGuid(n));

  REQUIRE(r.matched_count == 0);
  REQUIRE(r.fresh_count == 2);
  REQUIRE(r.instances.size() == 2);
  REQUIRE(r.orphaned_old.empty());
}

TEST_CASE("reconcile: mismatched cloud sizes throw", "[geometry][reconcile]") {
  auto old_l = makeLabels({1, 1, 1});
  auto new_l = makeLabels({1, 1});
  std::vector<PriorInstance> prior = {{1, 1, "g"}};
  std::map<uint32_t, uint32_t> new_sem = {{1, 1}};
  std::map<uint32_t, size_t> new_sizes = {{1, 2}};
  int n = 0;
  REQUIRE_THROWS_AS(reconcile_instance_identities(old_l.get(), *new_l, prior,
                                                  new_sem, new_sizes,
                                                  counterGuid(n)),
                    std::invalid_argument);
}

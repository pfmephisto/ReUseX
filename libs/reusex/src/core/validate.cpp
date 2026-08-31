// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/validate.hpp"
#include "core/ProjectDB.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace reusex::core {

bool ValidationReport::ok() const {
  return std::none_of(issues.begin(), issues.end(),
                      [](const ValidationIssue &i) {
                        return i.severity == ValidationSeverity::error;
                      });
}

size_t ValidationReport::error_count() const {
  return static_cast<size_t>(
      std::count_if(issues.begin(), issues.end(), [](const ValidationIssue &i) {
        return i.severity == ValidationSeverity::error;
      }));
}

size_t ValidationReport::warning_count() const {
  return static_cast<size_t>(
      std::count_if(issues.begin(), issues.end(), [](const ValidationIssue &i) {
        return i.severity == ValidationSeverity::warning;
      }));
}

namespace {

// Names of instance-label clouds — those that may have instance_materials
// links. We consider any cloud that has instances rows or material links.
std::vector<std::string> instance_cloud_names(const ProjectDB &db) {
  std::vector<std::string> result;
  for (const auto &name : db.list_point_clouds()) {
    bool has = false;
    try {
      has = !db.instances(name).empty() || !db.instance_materials(name).empty();
    } catch (const std::exception &) {
      has = false;
    }
    if (has)
      result.push_back(name);
  }
  return result;
}

} // namespace

void check_orphaned_passports(const ProjectDB &db,
                              std::vector<ValidationIssue> &out) {
  // Gather every material_guid referenced by any link.
  std::set<std::string> referenced;
  for (const auto &name : instance_cloud_names(db)) {
    for (const auto &[id, guid] : db.instance_materials(name))
      referenced.insert(guid);
  }

  for (const auto &guid : db.list_passport_guids()) {
    if (referenced.find(guid) == referenced.end()) {
      out.push_back({"orphaned_passport",
                     fmt::format("material passport {} is not linked to any "
                                 "instance",
                                 guid),
                     ValidationSeverity::warning});
    }
  }
}

void check_dangling_instance_materials(const ProjectDB &db,
                                       std::vector<ValidationIssue> &out) {
  std::set<std::string> passports;
  for (const auto &g : db.list_passport_guids())
    passports.insert(g);

  for (const auto &name : instance_cloud_names(db)) {
    // Set of valid instance ids for this cloud.
    std::set<uint32_t> instance_ids;
    for (const auto &r : db.instances(name))
      instance_ids.insert(r.instance_id);

    for (const auto &[iid, guid] : db.instance_materials(name)) {
      if (instance_ids.find(static_cast<uint32_t>(iid)) == instance_ids.end())
        out.push_back(
            {"dangling_instance_material",
             fmt::format("cloud '{}': link for instance {} has no instances "
                         "row (guid {})",
                         name, iid, guid),
             ValidationSeverity::error});
      if (passports.find(guid) == passports.end())
        out.push_back(
            {"dangling_instance_material",
             fmt::format("cloud '{}': instance {} links to missing passport {}",
                         name, iid, guid),
             ValidationSeverity::error});
    }
  }
}

void check_instances_without_label_defs(const ProjectDB &db,
                                        std::vector<ValidationIssue> &out) {
  for (const auto &name : instance_cloud_names(db)) {
    auto rows = db.instances(name);
    if (rows.empty())
      continue;
    std::map<int, std::string> defs;
    try {
      defs = db.label_definitions(name);
    } catch (const std::exception &) {
      // Treated as "no definitions": every instance is missing one.
    }
    for (const auto &r : rows) {
      if (defs.find(static_cast<int>(r.instance_id)) == defs.end())
        out.push_back({"instance_without_label_def",
                       fmt::format("cloud '{}': instance {} (guid {}) has no "
                                   "label_definitions entry",
                                   name, r.instance_id, r.guid),
                       ValidationSeverity::error});
    }
  }
}

void check_sibling_cloud_sizes(const ProjectDB &db,
                               std::vector<ValidationIssue> &out) {
  // The canonical parallel clouds for a scan; they are index-aligned.
  static const std::vector<std::string> siblings = {
      "cloud", "labels", "normals", "planes", "rooms", "instances"};

  auto summary = db.project_summary();
  std::map<std::string, size_t> counts;
  for (const auto &c : summary.clouds)
    counts[c.name] = c.point_count;

  // Reference size = "cloud" if present, else the first sibling present.
  std::string ref_name;
  size_t ref_size = 0;
  for (const auto &s : siblings) {
    auto it = counts.find(s);
    if (it != counts.end()) {
      ref_name = s;
      ref_size = it->second;
      break;
    }
  }
  if (ref_name.empty())
    return; // No sibling clouds present.

  for (const auto &s : siblings) {
    if (s == ref_name)
      continue;
    auto it = counts.find(s);
    if (it == counts.end())
      continue; // Missing sibling is not an error here.
    if (it->second != ref_size)
      out.push_back(
          {"sibling_size_mismatch",
           fmt::format("cloud '{}' has {} points but sibling '{}' has {} — "
                       "parallel clouds must be index-aligned",
                       s, it->second, ref_name, ref_size),
           ValidationSeverity::error});
  }
}

ValidationReport validate_project(const ProjectDB &db) {
  ValidationReport report;
  check_orphaned_passports(db, report.issues);
  check_dangling_instance_materials(db, report.issues);
  check_instances_without_label_defs(db, report.issues);
  check_sibling_cloud_sizes(db, report.issues);
  return report;
}

} // namespace reusex::core

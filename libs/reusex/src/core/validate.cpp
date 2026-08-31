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

// ── Stage input contracts (#222) ───────────────────────────────────────────

std::optional<PipelineStage> parse_pipeline_stage(std::string_view name) {
  if (name == "import")
    return PipelineStage::import;
  if (name == "optimize" || name == "register")
    return PipelineStage::optimize;
  if (name == "clouds")
    return PipelineStage::clouds;
  if (name == "planes")
    return PipelineStage::planes;
  if (name == "rooms")
    return PipelineStage::rooms;
  if (name == "instances")
    return PipelineStage::instances;
  if (name == "mesh")
    return PipelineStage::mesh;
  return std::nullopt;
}

std::string_view to_string(PipelineStage stage) {
  switch (stage) {
  case PipelineStage::import:
    return "import";
  case PipelineStage::optimize:
    return "optimize";
  case PipelineStage::clouds:
    return "clouds";
  case PipelineStage::planes:
    return "planes";
  case PipelineStage::rooms:
    return "rooms";
  case PipelineStage::instances:
    return "instances";
  case PipelineStage::mesh:
    return "mesh";
  }
  return "unknown";
}

std::vector<std::string> pipeline_stage_names() {
  return {"import", "optimize", "register",  "clouds",
          "planes", "rooms",    "instances", "mesh"};
}

namespace {

// point_count per named cloud from a single project_summary() call.
std::map<std::string, size_t> cloud_point_counts(const ProjectDB &db) {
  std::map<std::string, size_t> counts;
  auto summary = db.project_summary();
  for (const auto &c : summary.clouds)
    counts[c.name] = c.point_count;
  return counts;
}

// Require that a named cloud exists; append an error if not. Returns its point
// count when present (via out-param), leaving `size` untouched when absent.
bool require_cloud(const std::map<std::string, size_t> &counts,
                   const std::string &name, const char *stage,
                   std::vector<ValidationIssue> &out, size_t *size = nullptr) {
  auto it = counts.find(name);
  if (it == counts.end()) {
    out.push_back({"missing_stage_input",
                   fmt::format("stage '{}' requires cloud '{}' which is not "
                               "present in the project",
                               stage, name),
                   ValidationSeverity::error});
    return false;
  }
  if (size)
    *size = it->second;
  return true;
}

// Assert every present cloud in `names` has the same point count as the first
// present one; append an error per mismatch. Missing clouds are ignored here
// (require_cloud handles required presence).
void require_aligned(const std::map<std::string, size_t> &counts,
                     const std::vector<std::string> &names, const char *stage,
                     std::vector<ValidationIssue> &out) {
  std::string ref_name;
  size_t ref_size = 0;
  for (const auto &n : names) {
    auto it = counts.find(n);
    if (it == counts.end())
      continue;
    if (ref_name.empty()) {
      ref_name = n;
      ref_size = it->second;
      continue;
    }
    if (it->second != ref_size)
      out.push_back(
          {"stage_input_size_mismatch",
           fmt::format("stage '{}': cloud '{}' has {} points but '{}' has {} — "
                       "index-aligned inputs must match",
                       stage, n, it->second, ref_name, ref_size),
           ValidationSeverity::error});
  }
}

} // namespace

void check_stage_inputs(const ProjectDB &db, PipelineStage stage,
                        std::vector<ValidationIssue> &out) {
  const char *name = to_string(stage).data();

  switch (stage) {
  case PipelineStage::import:
    // import has no cloud/table prerequisites (it reads an external scan).
    break;

  case PipelineStage::optimize:
  case PipelineStage::clouds: {
    // Both consume stored sensor frames.
    auto summary = db.project_summary();
    if (summary.sensor_frames.total_count <= 0)
      out.push_back({"missing_stage_input",
                     fmt::format("stage '{}' requires stored sensor frames "
                                 "(run 'rux import' first)",
                                 name),
                     ValidationSeverity::error});
    break;
  }

  case PipelineStage::planes: {
    auto counts = cloud_point_counts(db);
    require_cloud(counts, "cloud", name, out);
    require_cloud(counts, "normals", name, out);
    require_aligned(counts, {"cloud", "normals"}, name, out);
    break;
  }

  case PipelineStage::rooms: {
    auto counts = cloud_point_counts(db);
    require_cloud(counts, "cloud", name, out);
    require_cloud(counts, "planes", name, out);
    require_cloud(counts, "plane_centroids", name, out);
    require_cloud(counts, "plane_normals", name, out);
    // cloud and planes are per-point and must be index-aligned; plane_centroids
    // / plane_normals are per-plane and intentionally shorter.
    require_aligned(counts, {"cloud", "planes"}, name, out);
    require_aligned(counts, {"plane_centroids", "plane_normals"}, name, out);
    break;
  }

  case PipelineStage::instances: {
    auto counts = cloud_point_counts(db);
    require_cloud(counts, "cloud", name, out);
    // Semantic labels: the default input cloud is "labels"; accept "planes" as
    // a fallback so the check is useful even when only geometric labels exist.
    if (counts.find("labels") == counts.end() &&
        counts.find("planes") == counts.end())
      out.push_back({"missing_stage_input",
                     fmt::format("stage '{}' requires a semantic label cloud "
                                 "('labels') to cluster into instances",
                                 name),
                     ValidationSeverity::error});
    require_aligned(counts, {"cloud", "labels"}, name, out);
    break;
  }

  case PipelineStage::mesh: {
    auto counts = cloud_point_counts(db);
    require_cloud(counts, "cloud", name, out);
    require_cloud(counts, "normals", name, out);
    require_cloud(counts, "rooms", name, out);
    require_cloud(counts, "planes", name, out);
    require_cloud(counts, "plane_centroids", name, out);
    require_cloud(counts, "plane_normals", name, out);
    require_aligned(counts, {"cloud", "normals", "rooms", "planes"}, name, out);
    require_aligned(counts, {"plane_centroids", "plane_normals"}, name, out);
    break;
  }
  }
}

ValidationReport validate_stage(const ProjectDB &db, PipelineStage stage) {
  ValidationReport report;
  check_stage_inputs(db, stage, report.issues);
  return report;
}

} // namespace reusex::core

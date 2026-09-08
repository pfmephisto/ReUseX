// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/stage_contract.hpp"

#include <algorithm>
#include <stdexcept>

namespace reusex::core {
namespace {

// ── The artifact registry ──────────────────────────────────────────────────
// Every named cloud / table / mesh a stage may consume or produce, with the
// index-alignment class it belongs to. Stages below refer to these by name;
// a name not listed here is rejected by the table-integrity unit test.
const std::vector<Artifact> &artifacts() {
  static const std::vector<Artifact> table = {
      {"sensor_frames", ArtifactKind::table, Alignment::none,
       "per-frame color/depth/confidence/pose/intrinsics rows"},
      {"segmentation_images", ArtifactKind::table, Alignment::none,
       "per-frame 2D semantic label images"},
      {"cloud", ArtifactKind::point_cloud, Alignment::per_point,
       "the fused scan cloud (PointXYZRGB)"},
      {"normals", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point surface normals (Normal)"},
      {"labels", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point semantic class labels (Label)"},
      {"planes", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point plane labels (Label)"},
      {"plane_centroids", ArtifactKind::point_cloud, Alignment::per_plane,
       "one centroid per detected plane (PointXYZ)"},
      {"plane_normals", ArtifactKind::point_cloud, Alignment::per_plane,
       "one normal per detected plane (Normal)"},
      {"rooms", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point room labels (Label)"},
      {"instances", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point instance labels (Label), plus the `instances` table"},
      {"mesh", ArtifactKind::mesh, Alignment::none,
       "the reconstructed watertight mesh"},
      {"textured_mesh", ArtifactKind::mesh, Alignment::none,
       "the texture-mapped mesh"},
      {"building_components", ArtifactKind::table, Alignment::none,
       "derived building components (windows, …)"},
  };
  return table;
}

// ── The stage table ────────────────────────────────────────────────────────
// Order is pipeline order and is load-bearing: an input may only be produced
// by a strictly earlier stage (asserted in test_stage_contract.cpp), which is
// also what makes the resolution-hint recursion terminate.
const std::vector<StageContract> &contracts() {
  static const std::vector<StageContract> table = {
      {PipelineStage::import,
       "import",
       {},
       "rux import rtabmap <scan.db>",
       "extract raw sensor frames from an external scan",
       // Reads an external scan file, so it has no in-project prerequisite.
       {},
       {"sensor_frames"}},

      {PipelineStage::optimize,
       "optimize",
       {"register"},
       "rux optimize",
       "refine the stored per-frame sensor poses",
       // Both pose refiners are depth-driven and need two frames to have any
       // relative pose to refine at all.
       {{{"sensor_frames"}, /*min_rows=*/2, /*min_depth_frames=*/2}},
       {"sensor_frames"}},

      {PipelineStage::clouds,
       "clouds",
       {},
       "rux create clouds",
       "back-project depth frames into a fused cloud",
       {{{"sensor_frames"}}},
       {"cloud", "normals"}},

      {PipelineStage::annotate,
       "annotate",
       {},
       "rux create annotate -n <model>",
       "ML semantic labelling of the stored frames",
       {{{"sensor_frames"}}},
       {"segmentation_images"}},

      {PipelineStage::project,
       "project",
       {},
       "rux create project",
       "project 2D semantic labels onto the 3D cloud",
       {{{"cloud"}}, {{"segmentation_images"}}},
       {"labels"}},

      {PipelineStage::planes,
       "planes",
       {},
       "rux create planes",
       "detect planar surfaces",
       {{{"cloud"}}, {{"normals"}}},
       {"planes", "plane_centroids", "plane_normals"}},

      {PipelineStage::rooms,
       "rooms",
       {},
       "rux create rooms",
       "partition the plane graph into rooms",
       {{{"cloud"}}, {{"planes"}}, {{"plane_centroids"}}, {{"plane_normals"}}},
       {"rooms"}},

      {PipelineStage::instances,
       "instances",
       {},
       "rux create instances",
       "split semantic labels into spatial instances",
       // `labels` is the default semantic input; the geometric `planes` cloud
       // is accepted as a fallback so the check stays useful on a project that
       // was never annotated.
       {{{"cloud"}}, {{"labels", "planes"}}},
       {"instances"}},

      {PipelineStage::mesh,
       "mesh",
       {},
       "rux create mesh",
       "solidify the cell complex into a room-partitioned mesh",
       {{{"cloud"}},
        {{"normals"}},
        {{"rooms"}},
        {{"planes"}},
        {{"plane_centroids"}},
        {{"plane_normals"}}},
       {"mesh"}},

      {PipelineStage::texture,
       "texture",
       {},
       "rux create texture",
       "texture-map the reconstructed mesh",
       {{{"mesh"}}, {{"cloud"}}, {{"sensor_frames"}}},
       {"textured_mesh"}},

      {PipelineStage::windows,
       "windows",
       {},
       "rux create windows",
       "derive window building components",
       {{{"cloud"}}, {{"labels"}}, {{"instances"}}, {{"mesh"}}},
       {"building_components"}},
  };
  return table;
}

} // namespace

const std::vector<StageContract> &stage_contracts() { return contracts(); }

const std::vector<Artifact> &pipeline_artifacts() { return artifacts(); }

const StageContract &stage_contract(PipelineStage stage) {
  for (const auto &entry : contracts())
    if (entry.stage == stage)
      return entry;
  // Unreachable for any enumerator. A new enumerator without a table row is a
  // programming error, so fail loudly rather than silently validating nothing.
  throw std::logic_error(
      "reusex::core: PipelineStage missing from the stage contract table");
}

const Artifact *find_artifact(std::string_view name) {
  for (const auto &entry : artifacts())
    if (entry.name == name)
      return &entry;
  return nullptr;
}

std::optional<PipelineStage> producing_stage(std::string_view artifact) {
  for (const auto &entry : contracts())
    if (std::find(entry.outputs.begin(), entry.outputs.end(), artifact) !=
        entry.outputs.end())
      return entry.stage;
  return std::nullopt;
}

std::optional<PipelineStage> parse_pipeline_stage(std::string_view name) {
  for (const auto &entry : contracts()) {
    if (entry.name == name)
      return entry.stage;
    if (std::find(entry.aliases.begin(), entry.aliases.end(), name) !=
        entry.aliases.end())
      return entry.stage;
  }
  return std::nullopt;
}

std::string_view to_string(PipelineStage stage) {
  return stage_contract(stage).name;
}

std::vector<std::string> pipeline_stage_names() {
  std::vector<std::string> names;
  for (const auto &entry : contracts()) {
    names.emplace_back(entry.name);
    for (const auto &alias : entry.aliases)
      names.emplace_back(alias);
  }
  return names;
}

} // namespace reusex::core

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/stage_parameters.hpp>

#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_instances.hpp>
#include <reusex/segmentation/segment_planes.hpp>
#include <reusex/segmentation/segment_rooms.hpp>

#include <fmt/format.h>

#include <stdexcept>

namespace reusex::pipeline {
namespace {

/// Widen a `float` option-struct field to the `double` the wire carries.
///
/// A plain `static_cast<double>(0.05F)` is 0.05000000074505806, which is what
/// a JSON encoder would faithfully print into a form field. Going through the
/// shortest round-trip decimal representation of the *float* gives back the
/// 0.05 the struct was written with, without inventing precision.
double widen(float value) { return std::stod(fmt::format("{}", value)); }

/// Every default below comes from a default-constructed option struct, so the
/// literals live in exactly one place — the struct — and this table cannot
/// drift away from what `run_stage()` actually falls back to.
/// @param default_value  taken as `float` because every fractional option in
///        the four structs is a `float`; passing a double here would quietly
///        admit a default this table cannot round-trip.
ParameterDescriptor num(std::string key, std::string label,
                        std::string description, float default_value,
                        std::optional<double> minimum = std::nullopt,
                        std::optional<double> maximum = std::nullopt) {
  return ParameterDescriptor{std::move(key),
                             ParameterType::number,
                             std::move(label),
                             std::move(description),
                             widen(default_value),
                             minimum,
                             maximum,
                             false};
}

ParameterDescriptor integer(std::string key, std::string label,
                            std::string description, long long default_value,
                            std::optional<double> minimum = std::nullopt,
                            std::optional<double> maximum = std::nullopt) {
  return ParameterDescriptor{std::move(key),   ParameterType::integer,
                             std::move(label), std::move(description),
                             default_value,    minimum,
                             maximum,          false};
}

ParameterDescriptor boolean(std::string key, std::string label,
                            std::string description, bool default_value) {
  return ParameterDescriptor{std::move(key),   ParameterType::boolean,
                             std::move(label), std::move(description),
                             default_value,    std::nullopt,
                             std::nullopt,     false};
}

ParameterDescriptor text(std::string key, std::string label,
                         std::string description, std::string default_value) {
  return ParameterDescriptor{std::move(key),
                             ParameterType::string,
                             std::move(label),
                             std::move(description),
                             std::move(default_value),
                             std::nullopt,
                             std::nullopt,
                             false};
}

/// A parameter with no neutral value: absent means "do not restrict".
ParameterDescriptor optional_of(ParameterType type, std::string key,
                                std::string label, std::string description) {
  return ParameterDescriptor{
      std::move(key),   type,         std::move(label), std::move(description),
      std::monostate{}, std::nullopt, std::nullopt,     false};
}

const std::vector<ParameterDescriptor> &clouds_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::ReconstructionParams d{};
    return std::vector<ParameterDescriptor>{
        num("resolution", "Voxel size [m]",
            "Voxel grid resolution used to downsample the fused cloud.",
            d.resolution),
        num("min_distance", "Minimum depth [m]",
            "Depth samples closer than this are discarded.", d.min_distance),
        num("max_distance", "Maximum depth [m]",
            "Depth samples farther than this are discarded.", d.max_distance),
        integer("sampling_factor", "Pixel subsampling",
                "Keep every Nth pixel of each depth frame.", d.sampling_factor),
        integer("confidence_threshold", "Minimum confidence",
                "Discard depth samples below this confidence level.",
                d.confidence_threshold),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &planes_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentPlanesOptions d{};
    std::vector<ParameterDescriptor> table{
        num("angle_threshold", "Angle threshold [deg]",
            "Maximum normal deviation for a point to join a plane.",
            d.angle_threshold, 0.0, 365.0),
        num("plane_dist_threshold", "Distance threshold [m]",
            "Maximum point-to-plane distance. Sending this key at all pins the "
            "threshold and switches off adaptive derivation for it.",
            d.plane_dist_threshold, 0.0, 1.0),
        integer("min_inliers", "Minimum cluster size",
                "Smallest accepted plane, in points. Sending this key at all "
                "pins it and switches off adaptive derivation for it.",
                d.min_inliers, 3.0, 1000000.0),
        num("radius", "Region-growing radius [m]",
            "Neighbourhood radius used while growing a plane.", d.radius, 0.0,
            5.0),
        num("interval_0", "Initial refit interval",
            "Initial interval between "
            "plane refits.",
            d.interval_0, 1.0, 10000.0),
        num("interval_factor", "Refit interval factor",
            "Multiplier applied to the refit interval after each refit.",
            d.interval_factor, 1.0, 10.0),
        boolean("adaptive", "Adaptive thresholds",
                "Derive the distance threshold (~3 sigma) and the minimum "
                "cluster size from measured cloud noise. Explicit values still "
                "win per parameter.",
                d.adaptive),
        integer("noise_seed", "Noise-estimator seed",
                "Deterministic seed for the noise estimator (STANDARDS §6).",
                static_cast<long long>(d.noise_seed), 0.0),
        optional_of(ParameterType::string, "filter", "Point filter",
                    "Filter expression restricting which points are "
                    "segmented. Empty means the whole cloud."),
    };
    // #214: these two are pinned by presence, not by value.
    for (auto &parameter : table)
      if (parameter.key == "plane_dist_threshold" ||
          parameter.key == "min_inliers")
        parameter.presence_sensitive = true;
    return table;
  }();
  return table;
}

const std::vector<ParameterDescriptor> &rooms_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentRoomsOptions d{};
    return std::vector<ParameterDescriptor>{
        num("grid_size", "Grid size [m]",
            "Spatial discretisation used to build the plane graph.",
            d.grid_size, 0.01, 10.0),
        num("resolution", "Leiden resolution",
            "Higher values produce more, smaller rooms.", d.resolution, 0.0,
            10.0),
        num("beta", "Leiden beta",
            "Randomness of the refinement phase; lower is more deterministic.",
            d.beta, 0.0, 1.0),
        integer("max_iter", "Maximum iterations",
                "Finite bound on the Leiden iteration count.", d.max_iter, -1.0,
                1000.0),
        integer("propagate_k", "Propagation neighbours",
                "Neighbours polled per point when propagating room labels off "
                "the sampled subset.",
                d.propagate_k, 1.0),
        num("propagate_max_radius", "Propagation radius [m]",
            "Points with no room label inside this radius stay unlabeled.",
            d.propagate_max_radius, 0.0, 10.0),
        optional_of(ParameterType::string, "filter", "Point filter",
                    "Filter expression restricting which points are "
                    "partitioned. Empty means the whole cloud."),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &instances_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentInstancesRequest d{};
    return std::vector<ParameterDescriptor>{
        text("semantic_cloud", "Semantic cloud",
             "Name of the semantic-label cloud to split into instances.",
             std::string(kDefaultSemanticCloud)),
        text("output_cloud", "Output cloud",
             "Name of the instance-label cloud to write.",
             std::string(kDefaultInstanceCloud)),
        num("cluster_tolerance", "Cluster tolerance [m]",
            "Euclidean distance threshold separating two instances.",
            d.cluster_tolerance, 0.01, 5.0),
        integer("min_cluster_size", "Minimum instance size",
                "Smallest accepted instance, in points.", d.min_cluster_size,
                1.0, 100000.0),
        integer("max_cluster_size", "Maximum instance size",
                "Largest accepted instance, in points.", d.max_cluster_size,
                10.0, 10000000.0),
        optional_of(ParameterType::integer_list, "labels", "Semantic labels",
                    "Restrict clustering to these semantic labels. Empty "
                    "means every label above 0."),
    };
  }();
  return table;
}

} // namespace

std::string_view to_string(ParameterType type) {
  switch (type) {
  case ParameterType::number:
    return "number";
  case ParameterType::integer:
    return "integer";
  case ParameterType::boolean:
    return "boolean";
  case ParameterType::string:
    return "string";
  case ParameterType::integer_list:
    return "integer_list";
  }
  throw std::logic_error("unhandled ParameterType");
}

const std::vector<ParameterDescriptor> &stage_parameters(JobStage stage) {
  switch (stage) {
  case JobStage::clouds:
    return clouds_parameters();
  case JobStage::planes:
    return planes_parameters();
  case JobStage::rooms:
    return rooms_parameters();
  case JobStage::instances:
    return instances_parameters();
  }
  throw std::logic_error("unhandled JobStage in stage_parameters()");
}

} // namespace reusex::pipeline

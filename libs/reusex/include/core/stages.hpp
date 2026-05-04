#pragma once

#include <string_view>

namespace reusex::core {

enum class Stage {
  idle,
  mesh_generation,
  importing_data,
  assembling_cloud,
  creating_material,
  retrieving_textures,
  computing_face_coverage,
  computing_room_probabilities,
  processing_batch,
  annotating_batches,
  projecting_labels,
  ray_tracing,
  region_growing,
  instance_clustering,
  creating_windows,
  room_segmentation,
};

// Convert Stage enum to human-readable string for logging/display
std::string_view to_string(Stage stage);

} // namespace reusex::core

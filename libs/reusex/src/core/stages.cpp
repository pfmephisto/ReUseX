// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/stages.hpp"

namespace reusex::core {

std::string_view to_string(Stage stage) {
  switch (stage) {
  case Stage::idle:
    return "Idle";
  case Stage::mesh_generation:
    return "Mesh Generation";
  case Stage::importing_data:
    return "Importing data";
  case Stage::assembling_cloud:
    return "Assembling cloud";
  case Stage::creating_material:
    return "Creating material";
  case Stage::retrieving_textures:
    return "Retrieving textures and cameras";
  case Stage::computing_face_coverage:
    return "Computing face coverage";
  case Stage::computing_room_probabilities:
    return "Computing room probabilities";
  case Stage::processing_batch:
    return "Processing batch";
  case Stage::annotating_batches:
    return "Annotating batches";
  case Stage::projecting_labels:
    return "Projecting labels";
  case Stage::ray_tracing:
    return "Ray tracing";
  case Stage::region_growing:
    return "Region Growing";
  case Stage::instance_clustering:
    return "Instance Clustering";
  case Stage::creating_windows:
    return "Creating Windows";
  case Stage::room_segmentation:
    return "Room Segmentation";
  default:
    return "Unknown Stage";
  }
}

} // namespace reusex::core

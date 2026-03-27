#pragma once

#include <string_view>

namespace ReUseX::core {

enum class Stage {
  Default,
  MeshGeneration,
  AssemblingCloud,
  CreatingMaterial,
  RetrievingTextures,
  ComputingFaceCoverage,
  ComputingRoomProbabilities,
  ProcessingBatch,
  AnnotatingBatches,
  ProjectingLabels,
  RayTracing,
  RegionGrowing,
};

// Convert Stage enum to human-readable string for logging/display
std::string_view to_string(Stage stage);

} // namespace ReUseX::core

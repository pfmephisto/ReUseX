// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/stages.hpp"

namespace ReUseX::core {

std::string_view to_string(Stage stage) {
  switch (stage) {
    case Stage::Default: return "Default";
    case Stage::MeshGeneration: return "Mesh Generation";
    case Stage::AssemblingCloud: return "Assembling cloud";
    case Stage::CreatingMaterial: return "Creating material";
    case Stage::RetrievingTextures: return "Retrieving textures and cameras";
    case Stage::ComputingFaceCoverage: return "Computing face coverage";
    case Stage::ComputingRoomProbabilities: return "Computing room probabilities";
    case Stage::ProcessingBatch: return "Processing batch";
    case Stage::AnnotatingBatches: return "Annotating batches";
    case Stage::ProjectingLabels: return "Projecting labels";
    case Stage::RayTracing: return "Ray tracing";
    case Stage::RegionGrowing: return "Region Growing";
    default: return "Unknown Stage";
  }
}

} // namespace ReUseX::core

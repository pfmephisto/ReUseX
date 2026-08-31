// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "processing_observer.hpp"
#include "view/viewer_types.hpp"

#include <reusex/geometry/BuildingComponent.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rux::view {

/// Generate all line shape names for a set of building components.
///
/// @param components Vector of building components
/// @return Vector of shape names (one per line segment across all components)
std::vector<std::string> component_line_names(
    const std::vector<reusex::geometry::BuildingComponent> &components);

/// Enqueue line-drawing tasks to render building component boundaries.
///
/// Each component's CoplanarPolygon is drawn as a closed loop of lines.
/// Windows are green, doors are cyan, walls are yellow.
///
/// @param components Vector of building components to render
/// @param observer Visualization observer for enqueueing viewer tasks
void add_component_lines(
    const std::vector<reusex::geometry::BuildingComponent> &components,
    rux::VizualizationObserver &observer);

/// Register keyboard callback to toggle building component visibility (key
/// 'w').
///
/// @param components Vector of building components
/// @param components_visible Shared flag tracking visibility state
/// @param observer Visualization observer for enqueueing viewer tasks
void register_component_toggle(
    const std::vector<reusex::geometry::BuildingComponent> &components,
    std::shared_ptr<bool> components_visible,
    rux::VizualizationObserver &observer);

/// Register help keyboard callback (key 'h').
///
/// @param clouds Vector of loaded clouds (for help message)
/// @param meshes Vector of loaded meshes (for help message)
/// @param label_clouds Vector of label clouds (for help message)
/// @param observer Visualization observer for enqueueing viewer tasks
void register_help_callback(const std::vector<LoadedCloud> &clouds,
                            const std::vector<LoadedMesh> &meshes,
                            const std::vector<LabelCloudInfo> &label_clouds,
                            bool has_components, bool has_panoramas,
                            rux::VizualizationObserver &observer);

} // namespace rux::view

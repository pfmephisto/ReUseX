// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ICP refine callback factory for POST /api/v1/posegraph/icp (#465).
//
// Lives in rux_lib (apps/rux/src/gui_icp.cpp), which links the `reusex`
// umbrella and therefore PCL. rux_gui_lib must NOT include this header —
// it would introduce a PCL dependency that bloats the light test binary.
// Only gui.cpp (rux_lib) and tests/unit/rux_app include it.

#include <gui/api.hpp>

namespace rux {

/// Build the IcpRefineFn callback that POST /api/v1/posegraph/icp uses.
///
/// Back-projects each frame's depth image into a PCL cloud (4px stride,
/// 0.3–4 m depth range, world space), then runs point-to-point ICP
/// (max 50 iterations, 0.5 m correspondence distance) seeded by the stored
/// world poses. Returns the refined relative pose T_to^{-1} @ T_delta @ T_from
/// plus RMS fitness and inlier fraction.
rux::gui::IcpRefineFn make_icp_refine_fn();

} // namespace rux

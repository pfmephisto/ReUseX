// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ICP refine callback factory for POST /api/v1/posegraph/icp (#465).
//
// Lives in ruxd_lib (apps/ruxd/src/icp.cpp), which links the `reusex`
// umbrella and therefore PCL. ruxd_api_lib must NOT include this header —
// it would introduce a PCL dependency that bloats the light test binary.
// Only ruxd's local-mode wiring (src/local.cpp) and tests/unit/ruxd include
// it.

#include <api/api.hpp>

namespace ruxd {

/// Build the IcpRefineFn callback that POST /api/v1/posegraph/icp uses.
///
/// Back-projects each frame's depth image into a PCL cloud (4px stride,
/// 0.3–4 m depth range, world space), then runs point-to-point ICP
/// (max 50 iterations, 0.5 m correspondence distance) seeded by the stored
/// world poses. Returns the refined relative pose T_to^{-1} @ T_delta @ T_from
/// plus RMS fitness and inlier fraction.
ruxd::api::IcpRefineFn make_icp_refine_fn();

} // namespace ruxd

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ICP-based relative-pose refinement for POST /api/v1/posegraph/icp (#465).
//
// The algorithm lives in the library (slam/frame_pair_icp.hpp), shared with
// the Qt client's pair strip; this file only adapts it to the IcpRefineFn
// callback the Server stores. LAYERING: it stays in rux_lib (which links the
// slam module and so PCL) — rux_gui_lib must NOT link it.

#include "gui_icp.hpp"

#include <reusex/slam/frame_pair_icp.hpp>

namespace rux {

gui::IcpRefineFn make_icp_refine_fn() {
  return [](const reusex::ProjectDB &db, int from_id,
            int to_id) -> gui::IcpRefineResult {
    const auto r = reusex::slam::refine_frame_pair_icp(db, from_id, to_id);
    gui::IcpRefineResult result;
    result.relative_pose = r.relative_pose;
    result.fitness = r.fitness;
    result.inlier_fraction = r.inlier_fraction;
    result.converged = r.converged;
    return result;
  };
}

} // namespace rux

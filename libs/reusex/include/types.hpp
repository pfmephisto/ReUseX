// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Backward-compatible umbrella header. Historically every TU included
// <reusex/types.hpp> to get both the PCL point-cloud aliases and the Eigen
// helpers. Those two groups now live in separate headers so a TU can pull in
// only what it uses (see issue #218). This umbrella keeps existing includers
// working by re-exporting both; new code should prefer the narrower headers:
//   - <reusex/types/point_types.hpp> for PCL point-cloud aliases
//   - <reusex/types/eigen_types.hpp> for the Eigen helpers
#include "reusex/core/stages.hpp"
#include "reusex/types/eigen_types.hpp"
#include "reusex/types/point_types.hpp"

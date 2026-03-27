// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <pcl/PolygonMesh.h>

namespace ReUseX::geometry {

/**
 * @brief Unweld (split) mesh vertices along sharp edges.
 *
 * Vertices shared by faces whose normals differ by more than
 * @p threshold_radians are duplicated so each smooth face group
 * gets its own copy. This produces correct per-face normals for
 * export to formats like OBJ, Speckle, or Rhino.
 *
 * @param mesh Input polygon mesh (const ref, not modified).
 * @param threshold_radians Maximum dihedral angle (in radians)
 *        between face normals for vertices to remain shared.
 *        0 = fully unweld, pi = preserve all sharing.
 * @return New polygon mesh with split vertices.
 */
pcl::PolygonMeshPtr unweld_mesh(const pcl::PolygonMesh &mesh,
                                float threshold_radians);

} // namespace ReUseX::geometry

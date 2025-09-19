// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <pcl/ModelCoefficients.h>

namespace ReUseX {
/** \brief Straighten a set of plane to be either vertical or horizontal.
 * \param coefficients the plane coefficients to be straightened
 * \param centroids the centroids of the planes
 */
void straiten(std::shared_ptr<pcl::ModelCoefficients> coefficients,
              pcl::PointCloud<pcl::PointXYZ>::Ptr centroids) {
  // TODO: Implement a function that snaps all planes to be either vertical or
  // horizontal while applying the rotation around the centroid of each plane.
}
} // namespace ReUseX

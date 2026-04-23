// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <pcl/PolygonMesh.h>

namespace reusex::geometry::cgal {

/// CGAL kernel for geometric computations
using K = CGAL::Simple_cartesian<double>;

/// CGAL 3D point type
using Point_3 = K::Point_3;

/// CGAL 3D vector type
using Vector_3 = K::Vector_3;

/// CGAL surface mesh type
using Mesh = CGAL::Surface_mesh<Point_3>;

/// Convert PCL PolygonMesh to CGAL Surface_mesh
/// @param pcl_mesh Input PCL mesh with PointXYZ vertices
/// @return CGAL Surface_mesh with equivalent topology
Mesh pcl_to_cgal_mesh(const pcl::PolygonMesh& pcl_mesh);

/// Convert CGAL Surface_mesh to PCL PolygonMesh
/// @param cgal_mesh Input CGAL mesh
/// @return PCL mesh with PointXYZ vertices
pcl::PolygonMesh cgal_to_pcl_mesh(const Mesh& cgal_mesh);

/// Convert Eigen Vector3d to CGAL Point_3
/// @param v Input Eigen vector
/// @return CGAL point with same coordinates
inline Point_3 eigen_to_cgal(const Eigen::Vector3d& v) {
    return Point_3(v.x(), v.y(), v.z());
}

/// Convert CGAL Point_3 to Eigen Vector3d
/// @param p Input CGAL point
/// @return Eigen vector with same coordinates
inline Eigen::Vector3d cgal_to_eigen(const Point_3& p) {
    return Eigen::Vector3d(CGAL::to_double(p.x()),
                          CGAL::to_double(p.y()),
                          CGAL::to_double(p.z()));
}

/// Convert CGAL Vector_3 to Eigen Vector3d
/// @param v Input CGAL vector
/// @return Eigen vector with same coordinates
inline Eigen::Vector3d cgal_to_eigen(const Vector_3& v) {
    return Eigen::Vector3d(CGAL::to_double(v.x()),
                          CGAL::to_double(v.y()),
                          CGAL::to_double(v.z()));
}

} // namespace reusex::geometry::cgal

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ReUseX/types.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <rtabmap/core/DBDriver.h>

#include <map>

namespace ReUseX::geometry {

pcl::TextureMesh::Ptr texture_mesh_with_cloud(pcl::PolygonMesh::Ptr mesh,
                                              CloudConstPtr cloud);

pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes);
} // namespace ReUseX::geometry

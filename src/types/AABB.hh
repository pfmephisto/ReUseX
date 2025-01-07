#pragma once

#include "types/Kernel.hh"
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>

namespace ReUseX
{
    using AABB = CGAL::Iso_cuboid_3<Kernel>;
    using Box = AABB;
} // namespace ReUseX

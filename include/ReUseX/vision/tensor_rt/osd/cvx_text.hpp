#pragma once
// Backward compatibility header — CvxText has been moved to vision/osd.
// New code should include <ReUseX/vision/osd/cvx_text.hpp> directly.
#include <ReUseX/vision/osd/cvx_text.hpp>

namespace ReUseX::vision::tensor_rt {
using CvxText = ReUseX::vision::osd::CvxText;
} // namespace ReUseX::vision::tensor_rt

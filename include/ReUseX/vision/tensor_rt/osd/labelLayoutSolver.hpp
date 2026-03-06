#pragma once
// Backward compatibility header — layout types have been moved to vision/osd.
// New code should include <ReUseX/vision/osd/labelLayoutSolver.hpp> directly.
#include <ReUseX/vision/osd/labelLayoutSolver.hpp>

namespace ReUseX::vision::tensor_rt {
using LayoutBox = ReUseX::vision::osd::LayoutBox;
using TextSize = ReUseX::vision::osd::TextSize;
using LayoutResult = ReUseX::vision::osd::LayoutResult;
using LayoutConfig = ReUseX::vision::osd::LayoutConfig;
using FlatUniformGrid = ReUseX::vision::osd::FlatUniformGrid;
using LabelLayoutSolver = ReUseX::vision::osd::LabelLayoutSolver;
} // namespace ReUseX::vision::tensor_rt

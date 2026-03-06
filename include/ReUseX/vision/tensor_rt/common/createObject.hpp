#pragma once
// Backward compatibility header — factory functions have been moved to
// vision/common. New code should include
// <ReUseX/vision/common/createObject.hpp> directly.
#include <ReUseX/vision/common/createObject.hpp>

namespace ReUseX::vision::tensor_rt::object {
using ReUseX::vision::common::object::createBox;
using ReUseX::vision::common::object::createDepthAnythingBox;
using ReUseX::vision::common::object::createDepthProBox;
using ReUseX::vision::common::object::createObbBox;
using ReUseX::vision::common::object::createPoseBox;
using ReUseX::vision::common::object::createPositionBox;
using ReUseX::vision::common::object::createSegmentationBox;
using ReUseX::vision::common::object::createTrackBox;
} // namespace ReUseX::vision::tensor_rt::object

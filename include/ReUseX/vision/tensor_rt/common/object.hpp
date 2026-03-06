#pragma once
// Backward compatibility header — types have been moved to vision/common.
// New code should include <ReUseX/vision/common/object.hpp> directly.
#include <ReUseX/vision/common/object.hpp>

namespace ReUseX::vision::tensor_rt::object {
using ObjectType = ReUseX::vision::common::object::ObjectType;
using Box = ReUseX::vision::common::object::Box;
using PosePoint = ReUseX::vision::common::object::PosePoint;
using Pose = ReUseX::vision::common::object::Pose;
using Obb = ReUseX::vision::common::object::Obb;
using SegmentMap = ReUseX::vision::common::object::SegmentMap;
using Segmentation = ReUseX::vision::common::object::Segmentation;
using Depth = ReUseX::vision::common::object::Depth;
using Track = ReUseX::vision::common::object::Track;
using DetectionBox = ReUseX::vision::common::object::DetectionBox;
using DetectionBoxArray = ReUseX::vision::common::object::DetectionBoxArray;
using ReUseX::vision::common::object::segmentMapToMat;
} // namespace ReUseX::vision::tensor_rt::object

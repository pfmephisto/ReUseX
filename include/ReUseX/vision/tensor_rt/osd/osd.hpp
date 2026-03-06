#pragma once
// Backward compatibility header — OSD functions have been moved to vision/osd.
// New code should include <ReUseX/vision/osd/osd.hpp> directly.
#include <ReUseX/vision/osd/osd.hpp>

namespace ReUseX::vision::tensor_rt {
using ReUseX::vision::osd::drawBaseInfoGeometry;
using ReUseX::vision::osd::drawDepth;
using ReUseX::vision::osd::drawObbBox;
using ReUseX::vision::osd::drawPolygon;
using ReUseX::vision::osd::drawPositionRectGeometry;
using ReUseX::vision::osd::drawPoseSkeleton;
using ReUseX::vision::osd::drawSegmentationMask;
using ReUseX::vision::osd::drawTrackHistoryPose;
using ReUseX::vision::osd::drawTrackTrace;
using ReUseX::vision::osd::osd;
using ReUseX::vision::osd::osd_new;
} // namespace ReUseX::vision::tensor_rt

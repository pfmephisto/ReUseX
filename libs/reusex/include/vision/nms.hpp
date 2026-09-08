// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <torch/torch.h>

namespace reusex::vision {

/** @brief Convert bounding boxes from (x1,y1,x2,y2) to (cx,cy,w,h) format. */
torch::Tensor xyxy_to_xywh(const torch::Tensor &x);

/** @brief Convert bounding boxes from (cx,cy,w,h) to (x1,y1,x2,y2) format. */
torch::Tensor xywh_to_xyxy(const torch::Tensor &x);

/** @brief Non-maximum suppression on bounding boxes.
 *
 * The algorithm is torchvision's reference CPU kernel, vendored under
 * BSD-3-Clause in `vision/third_party/torchvision_nms_kernel.hpp` (#141). This
 * wrapper owns the ReUseX-facing contract: CPU, float32, empty input tolerated.
 *
 * Boxes are suppressed when IoU is *strictly greater* than @p iou_threshold, so
 * a threshold of 1.0 suppresses nothing and ties at the threshold survive.
 * Equal scores are broken towards the lower index (the sort is stable).
 *
 * @param bboxes Bounding boxes tensor [N, 4] in xyxy format. CPU, float32.
 * @param scores Confidence scores tensor [N]. CPU, float32.
 * @param iou_threshold IoU threshold for suppression.
 * @return Indices of kept boxes, in descending score order (int64).
 * @throws c10::Error if the tensors are not CPU float32, or are malformed.
 */
torch::Tensor nms(const torch::Tensor &bboxes, const torch::Tensor &scores,
                  float iou_threshold = 0.45);

/** @brief YOLO-style non-maximum suppression with class-aware filtering.
 * @param predictions Raw model output [batch_size, 116, 8400].
 * @param confThreshold Confidence threshold for filtering.
 * @param iouThreshold IoU threshold for NMS.
 * @param maxDetections Maximum detections to keep per image.
 * @return Filtered detections [batch_size, maxDetections, 6+32].
 */
torch::Tensor non_max_suppression(torch::Tensor predictions,
                                  float confThreshold = 0.25,
                                  float iouThreshold = 0.45,
                                  int maxDetections = 300);

} // namespace reusex::vision

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
 * @param bboxes Bounding boxes tensor [N, 4] in xyxy format.
 * @param scores Confidence scores tensor [N].
 * @param iou_threshold IoU threshold for suppression.
 * @return Indices of kept boxes.
 */
// TODO: Replace custom NMS with torchvision library implementation
// category=Vision estimate=4h
// Current implementation is custom-written. Consider using official torchvision
// NMS: Reference:
// https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
// Benefits:
// 1. Optimized CPU/CUDA implementations available
// 2. Better maintained and tested by PyTorch team
// 3. Reduces custom code maintenance burden
// Trade-off: Adds torchvision as dependency (currently only use LibTorch)
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

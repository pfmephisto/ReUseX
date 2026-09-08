// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/nms.hpp"

#include "vision/third_party/torchvision_nms_kernel.hpp"

namespace reusex::vision {

torch::Tensor xyxy_to_xywh(const torch::Tensor &x) {
  auto y = torch::empty_like(x);
  y.index_put_({"...", 0}, (x.index({"...", 0}) + x.index({"...", 2})).div(2));
  y.index_put_({"...", 1}, (x.index({"...", 1}) + x.index({"...", 3})).div(2));
  y.index_put_({"...", 2}, x.index({"...", 2}) - x.index({"...", 0}));
  y.index_put_({"...", 3}, x.index({"...", 3}) - x.index({"...", 1}));
  return y;
}

torch::Tensor xywh_to_xyxy(const torch::Tensor &x) {
  auto y = torch::empty_like(x);
  auto dw = x.index({"...", 2}).div(2);
  auto dh = x.index({"...", 3}).div(2);
  y.index_put_({"...", 0}, x.index({"...", 0}) - dw);
  y.index_put_({"...", 1}, x.index({"...", 1}) - dh);
  y.index_put_({"...", 2}, x.index({"...", 0}) + dw);
  y.index_put_({"...", 3}, x.index({"...", 1}) + dh);
  return y;
}

torch::Tensor nms(const torch::Tensor &bboxes, const torch::Tensor &scores,
                  float iou_threshold) {
  // Short-circuit empty input before the precondition checks, preserving the
  // lenient contract this function had before #141: an empty box tensor is a
  // no-op regardless of its dtype or rank. The vendored kernel would instead
  // insist on a well-formed [0, 4] float tensor.
  if (bboxes.numel() == 0)
    return torch::empty({0}, bboxes.options().dtype(torch::kLong));

  // ReUseX narrows torchvision's contract to CPU float32. That is what every
  // call site produces (vision/libtorch/Yolo.cpp moves the model outputs to the
  // CPU before post-processing), and failing loudly here beats silently
  // dispatching a float64 copy of the whole detection set.
  TORCH_CHECK(bboxes.device().is_cpu(), "nms: bboxes must be on CPU");
  TORCH_CHECK(bboxes.dtype() == torch::kFloat32, "nms: bboxes must be float32");
  TORCH_CHECK(scores.device().is_cpu(), "nms: scores must be on CPU");
  TORCH_CHECK(scores.dtype() == torch::kFloat32, "nms: scores must be float32");

  // Algorithm proper: torchvision's reference CPU kernel, vendored verbatim.
  // Note the float -> double widening is exact and order-preserving, so the
  // strict `IoU > threshold` comparison decides identically either way.
  return torchvision::nms_kernel(bboxes, scores,
                                 static_cast<double>(iou_threshold));
}

torch::Tensor non_max_suppression(torch::Tensor predictions,
                                  float confThreshold, float iouThreshold,
                                  int maxDetections) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  auto bs = predictions.size(0); // batch size
  // YOLO-seg output: 4 bbox + nc classes + nm mask coefficients
  // nm=32 is the YOLO-seg default; nc is derived from the tensor shape
  constexpr int64_t kYoloMaskCoeffs = 32;
  auto nc = predictions.size(1) - 4 - kYoloMaskCoeffs; // num classes
  auto nm = kYoloMaskCoeffs;                           // num masks
  auto mi = 4 + nc;                                    // mask start index

  auto xc = predictions.index({Slice(), Slice(4, mi)}).amax(1) > confThreshold;

  predictions = predictions.transpose(-1, -2); // [bs, 8400, 116]
  predictions.index_put_(
      {"...", Slice({None, 4})},
      xywh_to_xyxy(predictions.index({"...", Slice(None, 4)})));

  torch::Tensor output =
      torch::zeros({bs, maxDetections, 6 + nm}, predictions.options());

  for (int xi = 0; xi < predictions.size(0); xi++) {
    auto x = predictions[xi];
    x = x.index({xc[xi]});

    auto x_split = x.split({4, nc, nm}, 1);
    auto box = x_split[0], cls = x_split[1], mask = x_split[2];

    auto [conf, j] = cls.max(1, true);

    x = torch::cat({box, conf, j.toType(torch::kFloat), mask}, 1);

    x = x.index({conf.view(-1) > confThreshold});
    int n = x.size(0);
    if (!n)
      continue;

    // Class offset for class-aware NMS: spatially separates each class's boxes
    // Value = 12 * default YOLO input size (640)
    constexpr float kClassOffset = 7680.0f;
    auto c = x.index({Slice(), Slice{5, 6}}) * kClassOffset;
    auto boxes = x.index({Slice(), Slice(None, 4)}) + c;
    auto scores = x.index({Slice(), 4});
    auto i = nms(boxes, scores, iouThreshold);
    i = i.index({Slice(None, maxDetections)});

    output.index_put_({xi, Slice(None, i.size(0))}, x.index({i}));
  }

  return output;
}

} // namespace reusex::vision

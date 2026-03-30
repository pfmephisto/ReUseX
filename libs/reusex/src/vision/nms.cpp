// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/nms.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ReUseX::vision {

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
  if (bboxes.numel() == 0)
    return torch::empty({0}, bboxes.options().dtype(torch::kLong));

  TORCH_CHECK(bboxes.device().is_cpu(), "nms: bboxes must be on CPU");
  TORCH_CHECK(bboxes.dtype() == torch::kFloat32, "nms: bboxes must be float32");
  TORCH_CHECK(scores.device().is_cpu(), "nms: scores must be on CPU");
  TORCH_CHECK(scores.dtype() == torch::kFloat32, "nms: scores must be float32");

  auto x1_t = bboxes.select(1, 0).contiguous();
  auto y1_t = bboxes.select(1, 1).contiguous();
  auto x2_t = bboxes.select(1, 2).contiguous();
  auto y2_t = bboxes.select(1, 3).contiguous();

  torch::Tensor areas_t = (x2_t - x1_t) * (y2_t - y1_t);

  auto order_t = std::get<1>(
      scores.sort(/*stable=*/true, /*dim=*/0, /* descending=*/true));

  auto ndets = bboxes.size(0);
  torch::Tensor suppressed_t =
      torch::zeros({ndets}, bboxes.options().dtype(torch::kByte));
  torch::Tensor keep_t =
      torch::zeros({ndets}, bboxes.options().dtype(torch::kLong));

  auto suppressed = suppressed_t.data_ptr<uint8_t>();
  auto keep = keep_t.data_ptr<int64_t>();
  auto order = order_t.data_ptr<int64_t>();
  auto x1 = x1_t.data_ptr<float>();
  auto y1 = y1_t.data_ptr<float>();
  auto x2 = x2_t.data_ptr<float>();
  auto y2 = y2_t.data_ptr<float>();
  auto areas = areas_t.data_ptr<float>();

  int64_t num_to_keep = 0;

  for (int64_t _i = 0; _i < ndets; _i++) {
    auto i = order[_i];
    if (suppressed[i] == 1)
      continue;
    keep[num_to_keep++] = i;
    auto ix1 = x1[i];
    auto iy1 = y1[i];
    auto ix2 = x2[i];
    auto iy2 = y2[i];
    auto iarea = areas[i];

    for (int64_t _j = _i + 1; _j < ndets; _j++) {
      auto j = order[_j];
      if (suppressed[j] == 1)
        continue;
      auto xx1 = std::max(ix1, x1[j]);
      auto yy1 = std::max(iy1, y1[j]);
      auto xx2 = std::min(ix2, x2[j]);
      auto yy2 = std::min(iy2, y2[j]);

      auto w = std::max(static_cast<float>(0), xx2 - xx1);
      auto h = std::max(static_cast<float>(0), yy2 - yy1);
      auto inter = w * h;
      auto ovr = inter / (iarea + areas[j] - inter);
      if (ovr > iou_threshold)
        suppressed[j] = 1;
    }
  }
  return keep_t.narrow(0, 0, num_to_keep);
}

torch::Tensor non_max_suppression(torch::Tensor predictions,
                                  float confThreshold, float iouThreshold,
                                  int maxDetections) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  auto bs = predictions.size(0);          // batch size
  // YOLO-seg output: 4 bbox + nc classes + nm mask coefficients
  // nm=32 is the YOLO-seg default; nc is derived from the tensor shape
  constexpr int64_t kYoloMaskCoeffs = 32;
  auto nc = predictions.size(1) - 4 - kYoloMaskCoeffs; // num classes
  auto nm = kYoloMaskCoeffs;                            // num masks
  auto mi = 4 + nc;                       // mask start index

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

} // namespace ReUseX::vision

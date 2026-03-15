// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/logging.hpp>
#include <ReUseX/vision/utils.hpp>

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <torch/torch.h>
#include <vector>

namespace ReUseX::vision {

float generate_scale(cv::Mat &image, const cv::Size &target_size,
                     bool scale_up) {
  int origin_w = image.cols;
  int origin_h = image.rows;

  int target_h = target_size.height; // 0 height
  int target_w = target_size.width;  // 1 width

  float ratio_h = static_cast<float>(target_h) / static_cast<float>(origin_h);
  float ratio_w = static_cast<float>(target_w) / static_cast<float>(origin_w);

  float resize_scale =
      scale_up ? std::max(ratio_h, ratio_w) : std::min(ratio_h, ratio_w);

  return resize_scale;
}

float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                const cv::Size &target_size) {
  if (input_image.cols == target_size.width &&
      input_image.rows == target_size.height) {
    if (input_image.data == output_image.data) {
      return 1.;
    } else {
      output_image = input_image.clone();
      return 1.;
    }
  }

  float resize_scale = generate_scale(input_image, target_size, false);
  int new_shape_w = std::round(input_image.cols * resize_scale);
  int new_shape_h = std::round(input_image.rows * resize_scale);
  float padw = (target_size.width - new_shape_w) / 2.;
  float padh = (target_size.height - new_shape_h) / 2.;

  int top = std::round(padh - 0.1);
  int bottom = std::round(padh + 0.1);
  int left = std::round(padw - 0.1);
  int right = std::round(padw + 0.1);

  cv::resize(input_image, output_image, cv::Size(new_shape_w, new_shape_h), 0,
             0, cv::INTER_AREA);

  cv::copyMakeBorder(output_image, output_image, top, bottom, left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(114., 114., 114));
  return resize_scale;
};

float cropbox(cv::Mat &input_image, cv::Mat &output_image,
              const cv::Size &target_size) {

  if (input_image.cols == target_size.width &&
      input_image.rows == target_size.height) {
    if (input_image.data == output_image.data) {
      return 1.;
    } else {
      output_image = input_image.clone();
      return 1.;
    }
  }

  float resize_scale = generate_scale(input_image, target_size, true);
  int new_shape_w = std::round(input_image.cols * resize_scale);
  int new_shape_h = std::round(input_image.rows * resize_scale);

  cv::resize(input_image, output_image, cv::Size(new_shape_w, new_shape_h), 0,
             0, cv::INTER_AREA);

  int x = (output_image.cols - target_size.width) / 2;
  int y = (output_image.rows - target_size.height) / 2;

  cv::Rect roi(x, y, target_size.width, target_size.height);
  output_image = output_image(roi).clone();

  return resize_scale;
};

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

// TODO: Replace custom NMS with torchvision library implementation
// category=Vision estimate=4h
// Current implementation is custom-written. Consider using official torchvision NMS:
// Reference: https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
// Benefits:
// 1. Optimized CPU/CUDA implementations available
// 2. Better maintained and tested by PyTorch team
// 3. Reduces custom code maintenance burden
// Trade-off: Adds torchvision as dependency (currently only use LibTorch)
torch::Tensor nms(const torch::Tensor &bboxes, const torch::Tensor &scores,
                  float iou_threshold) {
  if (bboxes.numel() == 0)
    return torch::empty({0}, bboxes.options().dtype(torch::kLong));

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

torch::Tensor non_max_suppression(torch::Tensor &predictions,
                                  float confThreshold, float iouThreshold,
                                  int maxDetections) {
  // INFO: predictions shape: [batch_size, 116, 8400]
  // 4(box x,y,w,h)+80(class weight)+32(mask weights)=116
  // 8400 boxes per image
  // Output shape: [batch_size, num_detections(max), 6 + 32]
  // 6 = 4(box x1,y1,x2,y2)+1(conf)+1(class) +32(mask weights)

  using torch::indexing::None;
  using torch::indexing::Slice;

  auto bs = predictions.size(0);          // batch size
  auto nc = predictions.size(1) - 4 - 32; // num classes (32 mask weights)
  auto nm = predictions.size(1) - nc - 4; // num masks
  auto mi = 4 + nc;                       // mask start index

  // ReUseX::core::debug("bs: {}, nc: {}, nm: {}, mi:{}", bs, nc, nm, mi);
  auto xc = predictions.index({Slice(), Slice(4, mi)}).amax(1) > confThreshold;

  // ReUseX::core::debug("xc shape: [{}]", fmt::join(xc.sizes(), ", "));

  predictions = predictions.transpose(-1, -2); // [bs, 8400, 116]
  predictions.index_put_({"...", Slice({None, 4})},
                         xywh_to_xyxy(predictions.index({"...", Slice(None, 4)})));

  torch::Tensor output =
      torch::zeros({bs, maxDetections, 6 + nm}, predictions.options());

  // Per batch
  for (int xi = 0; xi < predictions.size(0); xi++) {
    auto x = predictions[xi]; // [8400, 116]:
    x = x.index({xc[xi]});    // Filter by confidence

    // Split into boxes, class scores and masks
    auto x_split = x.split({4, nc, nm}, 1);
    auto box = x_split[0], cls = x_split[1], mask = x_split[2];

    auto [conf, j] = cls.max(1, true); // confidence and index of class

    x = torch::cat({box, conf, j.toType(torch::kFloat), mask},
                   1); // [num, 4+1+1+32] = [num, 38]

    // filter by confidence
    x = x.index({conf.view(-1) > confThreshold});
    int n = x.size(0);
    if (!n)
      continue; // no boxes to process

    // INFO: NMS
    // Here, each class index is multiplied by 7680.
    // This is a large constant chosen to offset boxes of different classes in
    // coordinate space so that boxes from different classes don’t overlap
    // during Non-Max Suppression (NMS).
    auto c = x.index({Slice(), Slice{5, 6}}) * 7680; // [num, 1]
    auto boxes = x.index({Slice(), Slice(None, 4)}) + c;
    auto scores = x.index({Slice(), 4});
    auto i = nms(boxes, scores, iouThreshold); // indices to keep
    i = i.index({Slice(None, maxDetections)});

    output.index_put_({xi, Slice(None, i.size(0))}, x.index({i}));
  }

  //// Convert boxes back to xywh
  // output.index_put_({"...", Slice({None, 4})},
  //                   xyxy_to_xywh(output.index({"...", Slice(None, 4)})));

  return output;
}

} // namespace ReUseX::vision

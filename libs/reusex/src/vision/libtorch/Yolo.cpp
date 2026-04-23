// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/libtorch/Data.hpp"
#include "vision/libtorch/Yolo.hpp"
#include "vision/nms.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <torch/script.h>
#include <torch/torch.h>

namespace reusex::vision::libtorch {

namespace {

struct BoundingBox {
  int x1, y1, x2, y2;
  int width() const { return std::max(0, x2 - x1); }
  int height() const { return std::max(0, y2 - y1); }
  bool isValid() const { return width() > 0 && height() > 0; }
  cv::Rect toRect() const { return cv::Rect(x1, y1, width(), height()); }
};

BoundingBox clipBoundingBox(torch::Tensor box, int imgWidth, int imgHeight) {
  BoundingBox bbox;
  bbox.x1 = std::max(0, box[0].item<int>());
  bbox.y1 = std::max(0, box[1].item<int>());
  bbox.x2 = std::min(imgWidth, box[2].item<int>());
  bbox.y2 = std::min(imgHeight, box[3].item<int>());
  return bbox;
}

cv::Mat resizeMask(torch::Tensor maskTensor, cv::Size targetSize) {
  auto maskCpu = maskTensor.to(torch::kCPU).contiguous();
  const int mask_h = static_cast<int>(maskCpu.size(0));
  const int mask_w = static_cast<int>(maskCpu.size(1));
  cv::Mat src(mask_h, mask_w, CV_32F, maskCpu.data_ptr<float>());
  cv::Mat dst;
  cv::resize(src, dst, targetSize, 0, 0, cv::INTER_NEAREST);
  return dst;
}

std::pair<torch::Tensor, torch::Tensor> processMasks(torch::Tensor keep,
                                                      torch::Tensor p2) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  auto const batch_size = keep.size(0);
  auto max_num_detections = keep.size(1);

  auto const nm = p2.size(1);   // number of prototype masks
  auto const ph = p2.size(2);   // prototype height
  auto const pw = p2.size(3);   // prototype width

  auto p2_flat = p2.view({batch_size, nm, -1});
  auto mask_weights = keep.index({Slice(), Slice(), Slice(6, None)});

  auto masks = torch::bmm(mask_weights, p2_flat);
  masks = masks.view({batch_size, max_num_detections, ph, pw});
  masks = masks.sigmoid();

  return {masks, keep};
}

cv::Mat createLabelImage(torch::Tensor keep, torch::Tensor masks,
                          size_t batchIndex, int imgSize,
                          float confThreshold) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  cv::Mat labelImg = cv::Mat::zeros(imgSize, imgSize, CV_16U);
  auto max_num_detections = keep.size(1);

  for (size_t j = 0; j < static_cast<size_t>(max_num_detections); ++j) {
    auto conf = keep[batchIndex][j][4].item<float>();

    if (conf < confThreshold)
      continue;

    auto id = keep[batchIndex][j][5].item<int>();
    auto box = keep[batchIndex][j].index({Slice(None, 4)});

    cv::Mat mask = resizeMask(masks[batchIndex][j], cv::Size(imgSize, imgSize));
    auto bbox = clipBoundingBox(box, labelImg.cols, labelImg.rows);

    if (!bbox.isValid())
      continue;

    labelImg(bbox.toRect()).setTo(id + 1, mask(bbox.toRect()) > 0.5);
  }

  return labelImg;
}

} // anonymous namespace

LibTorchYolo::LibTorchYolo(const std::filesystem::path &model_path,
                           bool use_cuda)
    : device_(torch::kCPU) {
  if (use_cuda && torch::cuda::is_available()) {
    device_ = torch::Device(torch::kCUDA);
    reusex::debug("LibTorchYolo using CUDA device");
  } else {
    if (use_cuda && !torch::cuda::is_available())
      reusex::warn("CUDA not available, falling back to CPU");
    reusex::debug("LibTorchYolo using CPU device");
  }

  model_ = torch::jit::load(model_path.string(), device_);
  model_.eval();
  model_.to(device_, torch::kFloat32);

  reusex::info("Loaded LibTorch YOLO model from {}", model_path);
}

std::unique_ptr<LibTorchYolo>
LibTorchYolo::create(const std::filesystem::path &model_path, bool use_cuda) {
  return std::make_unique<LibTorchYolo>(model_path, use_cuda);
}

torch::Tensor LibTorchYolo::images_to_tensor(
    const std::vector<const LibTorchData *> &inputs) const {
  std::vector<torch::Tensor> tensors;
  tensors.reserve(inputs.size());

  for (const auto *data : inputs) {
    const cv::Mat &img = data->image;
    torch::Tensor t =
        torch::from_blob(img.data, {img.rows, img.cols, 3}, torch::kByte)
            .clone();
    t = t.toType(torch::kFloat32).div(255);
    t = t.index({torch::indexing::Slice(), torch::indexing::Slice(),
                 torch::tensor({2, 1, 0})}); // BGR -> RGB
    t = t.permute({2, 0, 1}); // HWC -> CHW
    tensors.push_back(t);
  }

  if (tensors.size() > 1) {
    auto ref_shape = tensors[0].sizes();
    for (size_t k = 1; k < tensors.size(); ++k) {
      if (tensors[k].sizes() != ref_shape)
        throw std::runtime_error(
            "LibTorchYolo: image " + std::to_string(k) +
            " has different shape than image 0 — cannot batch");
    }
  }

  return torch::stack(tensors).to(device_);
}

std::vector<IDataset::Pair>
LibTorchYolo::forward(const std::span<IDataset::Pair> &input) {
  if (input.empty())
    return {};

  // Cast inputs to LibTorchData*
  std::vector<const LibTorchData *> lt_inputs;
  lt_inputs.reserve(input.size());
  for (const auto &[data, idx] : input) {
    auto *lt_data = dynamic_cast<const LibTorchData *>(data.get());
    if (!lt_data)
      throw std::runtime_error("LibTorchYolo::forward: input is not LibTorchData");
    lt_inputs.push_back(lt_data);
  }

  // Build batched tensor [B, 3, H, W]
  torch::Tensor batch_tensor = images_to_tensor(lt_inputs);

  // Run model inference
  auto output = model_.forward({batch_tensor});

  if (!output.isTuple()) {
    throw std::runtime_error(
        "LibTorchYolo: Model output is not a tuple. "
        "Expected YOLO segmentation model (yolo*-seg.pt).");
  }

  auto out_tuple = output.toTuple();
  auto elements = out_tuple->elements();

  if (elements.size() < 2) {
    throw std::runtime_error(
        "LibTorchYolo: Model output tuple has insufficient elements. "
        "Expected at least 2 elements (predictions and masks) for YOLO "
        "segmentation model (yolo*-seg.pt).");
  }

  auto p1 = elements[0].toTensor().cpu();
  auto p2 = elements[1].toTensor().cpu();

  // Get thresholds from first input (all should be the same)
  float conf_threshold = lt_inputs[0]->confidence_threshold;
  float iou_threshold = lt_inputs[0]->iou_threshold;
  int max_detections = lt_inputs[0]->max_detections;

  auto keep = non_max_suppression(p1, conf_threshold, iou_threshold,
                                  max_detections);

  auto actual_batch_size = static_cast<size_t>(keep.size(0));
  auto [masks, processed_keep] = processMasks(keep, p2);

  // Build output pairs with new LibTorchData containing label images
  std::vector<IDataset::Pair> results(actual_batch_size);

  for (size_t i = 0; i < actual_batch_size; ++i) {
    auto res_ptr = std::make_unique<LibTorchData>();
    res_ptr->original_size = lt_inputs[i]->original_size;
    res_ptr->target_size = lt_inputs[i]->target_size;
    res_ptr->letterbox_scale = lt_inputs[i]->letterbox_scale;
    res_ptr->label_image = createLabelImage(
        processed_keep, masks, i, lt_inputs[i]->target_size, conf_threshold);

    results[i].first = std::move(res_ptr);
    results[i].second = input[i].second;
  }

  return results;
}

} // namespace reusex::vision::libtorch

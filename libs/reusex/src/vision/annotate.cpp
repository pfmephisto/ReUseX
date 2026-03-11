// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/annotate.hpp>
#include <ReUseX/vision/libtorch/Dataset.hpp>
#include <ReUseX/vision/utils.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <fmt/std.h>
#include <spdmon/spdmon.hpp>

#include <torch/script.h>
#include <torch/torch.h>

#include <opencv2/core.hpp>
// For visualization/debugging purposes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/colors.h>

#include <spdlog/spdlog.h>

#include <range/v3/all.hpp>
namespace ReUseX::vision {

namespace {

/**
 * @brief Bounding box representation for object detection.
 */
struct BoundingBox {
  int x1, y1, x2, y2;
  int width() const { return std::max(0, x2 - x1); }
  int height() const { return std::max(0, y2 - y1); }
  bool isValid() const { return width() > 0 && height() > 0; }
  cv::Rect toRect() const { return cv::Rect(x1, y1, width(), height()); }
};

/**
 * @brief Clip bounding box to image dimensions.
 * @param box Bounding box tensor [x1, y1, x2, y2].
 * @param imgWidth Image width.
 * @param imgHeight Image height.
 * @return Clipped bounding box.
 */
BoundingBox clipBoundingBox(torch::Tensor box, int imgWidth, int imgHeight) {
  BoundingBox bbox;
  bbox.x1 = std::max(0, box[0].item<int>());
  bbox.y1 = std::max(0, box[1].item<int>());
  bbox.x2 = std::min(imgWidth, box[2].item<int>());
  bbox.y2 = std::min(imgHeight, box[3].item<int>());
  return bbox;
}

/**
 * @brief Resize segmentation mask to target size.
 * @param maskTensor Input mask tensor (160x160).
 * @param targetSize Target size for resized mask.
 * @return Resized mask as OpenCV Mat.
 */
cv::Mat resizeMask(torch::Tensor maskTensor, cv::Size targetSize) {
  cv::Mat mask(160, 160, CV_32F, maskTensor.cpu().data_ptr<float>());
  cv::resize(mask, mask, targetSize, 0, 0, cv::INTER_NEAREST);
  return mask;
}

/**
 * @brief Select computation device (CUDA or CPU).
 * @param isCuda Whether to attempt using CUDA.
 * @return Selected torch device.
 */
torch::Device selectDevice(bool isCuda) {
  torch::Device device =
      (isCuda && torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU;
  if (isCuda && !torch::cuda::is_available())
    spdlog::warn("CUDA is not available. Using CPU for inference.");
  spdlog::debug("Using device: {}", device.is_cuda() ? "CUDA" : "CPU");
  return device;
}

/**
 * @brief Load TorchScript model from file.
 * @param modelPath Path to model file.
 * @param device Device to load model on.
 * @return Loaded model in evaluation mode.
 */
torch::jit::script::Module loadModel(const std::filesystem::path &modelPath,
                                     torch::Device device) {
  auto model = torch::jit::load(modelPath.string(), device);
  model.eval();
  model.to(device, torch::kFloat32);
  return model;
}

/**
 * @brief Process mask predictions from YOLO model.
 * @param keep Filtered detection tensor.
 * @param p2 Proto masks from model.
 * @return Pair of (processed masks, keep tensor).
 */
std::pair<torch::Tensor, torch::Tensor> processMasks(torch::Tensor keep,
                                                     torch::Tensor p2) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  auto const batch_size = keep.size(0);
  auto max_num_detections = keep.size(1);

  auto p2_flat = p2.view({batch_size, 32, -1});
  auto mask_weights = keep.index({Slice(), Slice(), Slice(6, None)});

  auto masks = torch::bmm(mask_weights, p2_flat);
  masks = masks.view({batch_size, max_num_detections, 160, 160});
  masks = masks.sigmoid();

  return {masks, keep};
}

/**
 * @brief Create label image from detection masks.
 *
 * Generates a label image where each pixel is assigned the class ID
 * of the detected object it belongs to.
 *
 * @param keep Filtered detection tensor.
 * @param masks Processed mask tensors.
 * @param batchIndex Index of image in batch.
 * @param imgSize Size of output label image.
 * @return Label image with class IDs.
 */
cv::Mat createLabelImage(torch::Tensor keep, torch::Tensor masks,
                         size_t batchIndex, int imgSize) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  // cv::Mat labelImg = cv::Mat::zeros(imgSize, imgSize, CV_32S);
  cv::Mat labelImg = cv::Mat::zeros(imgSize, imgSize, CV_16U);
  auto max_num_detections = keep.size(1);

  for (size_t j = 0; j < static_cast<size_t>(max_num_detections); ++j) {
    auto conf = keep[batchIndex][j][4].item<float>();

    if (conf < 0.5)
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

template <typename T>
cv::Mat drawPred(cv::Mat const &img_, T output, T labels) {
  using torch::indexing::None;
  using torch::indexing::Slice;

  std::vector<std::string> classNames = {
      "person",        "bicycle",      "car",
      "motorcycle",    "airplane",     "bus",
      "train",         "truck",        "boat",
      "traffic light", "fire hydrant", "stop sign",
      "parking meter", "bench",        "bird",
      "cat",           "dog",          "horse",
      "sheep",         "cow",          "elephant",
      "bear",          "zebra",        "giraffe",
      "backpack",      "umbrella",     "handbag",
      "tie",           "suitcase",     "frisbee",
      "skis",          "snowboard",    "sports ball",
      "kite",          "baseball bat", "baseball glove",
      "skateboard",    "surfboard",    "tennis racket",
      "bottle",        "wine glass",   "cup",
      "fork",          "knife",        "spoon",
      "bowl",          "banana",       "apple",
      "sandwich",      "orange",       "broccoli",
      "carrot",        "hot dog",      "pizza",
      "donut",         "cake",         "chair",
      "couch",         "potted plant", "bed",
      "dining table",  "toilet",       "tv",
      "laptop",        "mouse",        "remote",
      "keyboard",      "cell phone",   "microwave",
      "oven",          "toaster",      "sink",
      "refrigerator",  "book",         "clock",
      "vase",          "scissors",     "teddy bear",
      "hair drier",    "toothbrush"};

  using pcl::GlasbeyLUT;

  auto colors = [](int i) {
    auto c = GlasbeyLUT::at(i % GlasbeyLUT::size());
    return cv::Scalar(c.r, c.g, c.g);
  };

  cv::Mat img = img_.clone();

  auto num_detections = output.size(0);
  for (int i = 0; i < num_detections; i++) {

    auto conf = output[i][4].template item<float>();
    if (conf < 0.5)
      continue;

    auto id = output[i][5].template item<int>();

    auto box = output[i].index({Slice(None, 4)});

    auto bbox = clipBoundingBox(box, img.cols, img.rows);
    if (!bbox.isValid())
      continue;

    cv::Rect rect = bbox.toRect();

    cv::Mat mask = resizeMask(labels[i], cv::Size(img.cols, img.rows));
    // img(rect).setTo(colors(id), mask(rect) > 0.5);

    cv::Mat roi = img(rect);

    // Blend the color
    cv::Mat color_rect(roi.size(), roi.type(), colors(id));

    cv::Mat blended;
    cv::addWeighted(roi, 1.0, color_rect, 0.5, 0.0, blended);

    // Copy only masked pixels
    blended.copyTo(roi, mask(rect) > 0.5);

    // Bounding box
    cv::rectangle(img, rect, colors(id), 2, 8);

    // Text label
    std::string label = fmt::format("{}: {:.2f}", classNames[id], conf);
    int baseLine;
    cv::Size textSize =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    int y1 = std::max(bbox.y1, textSize.height);

    cv::putText(img, label, cv::Point(bbox.x1, y1), cv::FONT_HERSHEY_SIMPLEX, 1,
                colors(id), 2);
  }
  return img;
}

void applyGlasberyColorMap(const cv::Mat &input, cv::Mat &output) {

  cv::Mat temp = cv::Mat::zeros(input.size(), CV_8UC4);
  temp.copySize(input);

  for (int r = 0; r < input.rows; ++r) {
    for (int c = 0; c < input.cols; ++c) {
      int l = input.at<int>(r, c);
      auto color = pcl::GlasbeyLUT::at(l % pcl::GlasbeyLUT::size());
      temp.at<cv::Vec4b>(r, c) =
          cv::Vec4b(color.b, color.g, color.r, l == 0 ? 0 : 255);
    }
  }

  output = temp;
}

auto annotateRTABMap(const std::filesystem::path &dbPath,
                     const std::filesystem::path &modelPath, bool isCuda)
    -> int {
  spdlog::trace("calling annotateRTABMap");

  torch::manual_seed(1);

  torch::Device device = selectDevice(isCuda);
  auto model = loadModel(modelPath, device);

  auto dataset = ReUseX::vision::libtorch::TorchDataset(dbPath);
  auto mapped_dataset = dataset.map(torch::data::transforms::Stack<>());

  const size_t batch_size = 16;
  size_t num_batches = (dataset.size().value() + batch_size - 1) / batch_size;

  auto dataloader =
      torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
          std::move(mapped_dataset),
          torch::data::DataLoaderOptions().batch_size(batch_size).workers(5));

  {
    spdlog::info("Starting annotation of {} batches", num_batches);
    auto logger = spdmon::LoggerProgress("Processing batch", num_batches);

    for (torch::data::Example<> &batch : *dataloader) {
      torch::Tensor data = batch.data.to(device);
      torch::Tensor node_ids = batch.target.cpu();

      auto output = model.forward({data});

      if (output.isTuple() == false) {
        spdlog::error("Model output is not a tuple as expected.");
        spdlog::info("Have you used the correct model? This script expects the "
                     "yolo segmentation variant.");
        return -1;
      }

      auto out_tuple = output.toTuple();
      auto elements = out_tuple->elements();

      auto p1 = elements[0].toTensor().cpu();
      auto p2 = elements[1].toTensor().cpu();

      auto keep = non_max_suppression(p1);

      auto const actual_batch_size = keep.size(0);
      auto [masks, processed_keep] = processMasks(keep, p2);

      std::vector<cv::Mat> labels(actual_batch_size);
      for (size_t i = 0; i < static_cast<size_t>(actual_batch_size); ++i) {
        labels[i] = createLabelImage(processed_keep, masks, i, 640);
      }

      if (false) {
        for (size_t i = 0; i < static_cast<size_t>(actual_batch_size); ++i) {
          cv::Mat img = cv::Mat(
              640, 640, CV_32FC3,
              data[i].cpu().permute({1, 2, 0}).contiguous().data_ptr<float>());
          img.convertTo(img, CV_8UC3, 255.0);
          cv::imshow("Image", drawPred(img, processed_keep[i], masks[i]));
          cv::waitKey(0);
        }
      }

      dataset.save(labels, node_ids);

      ++logger;
    }
  }

  return 0;
}

} // namespace ReUseX::vision

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/annotate.hpp>
#include <ReUseX/vision/Dataset.hpp>
#include <ReUseX/vision/utils.hpp>

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

    // Clip boxes to image size
    int x1 = std::max(0, box[0].template item<int>());
    int y1 = std::max(0, box[1].template item<int>());
    int x2 = std::min(img.cols, box[2].template item<int>());
    int y2 = std::min(img.rows, box[3].template item<int>());

    int width = std::max(0, x2 - x1);
    int height = std::max(0, y2 - y1);

    if (width == 0 || height == 0)
      continue; // skip invalid boxes

    cv::Rect rect(x1, y1, width, height);

    cv::Mat mask(160, 160, CV_32F, labels[i].cpu().template data_ptr<float>());
    cv::resize(mask, mask, cv::Size(img.cols, img.rows));
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
    y1 = std::max(y1, textSize.height);

    // cv::rectangle(frame, cv::Point(x1, y1 - int(1.5 * textSize.height)),
    //               cv::Point(x1 + int(1.5 * textSize.width), y1 + baseLine),
    //               cv::Scalar(0, 255, 0), cv::FILLED);

    cv::putText(img, label, cv::Point(x1, y1), cv::FONT_HERSHEY_SIMPLEX, 1,
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

auto annotateRTABMap(const fs::path &dbPath, const fs::path &modelPath,
                     bool isCuda) -> int {
  spdlog::trace("calling annotateRTABMap");

  torch::manual_seed(1);

  // INFO: Initialize the Yolo model if image segmentation is enabled
  torch::Device device =
      (isCuda && torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU;
  if (isCuda && !torch::cuda::is_available())
    spdlog::warn("CUDA is not available. Using CPU for inference.");
  spdlog::debug("Using device: {}", device.is_cuda() ? "CUDA" : "CPU");

  auto model = torch::jit::load(modelPath.string(), device);
  model.eval();
  model.to(device, torch::kFloat32);

  auto dataset = ReUseX::vision::Dataset(dbPath);
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
      torch::Tensor node_ids = batch.target.cpu(); //.to(device);

      auto output = model.forward({data});
      auto out_tuple = output.toTuple();
      auto elements = out_tuple->elements();

      auto p1 = elements[0].toTensor().cpu(); // p1 shape: [16, 116, 8400]
      auto p2 = elements[1].toTensor().cpu(); // p2 shape: [16, 32, 160, 160]

      // TODO:: NMS and postprocessing
      using torch::indexing::None;
      using torch::indexing::Slice;

      // INFO: Model output details
      // Payload 1: 4(box x,y,w,h)+80(class weight)+32(mask weights)=116
      // 8400 boxes per image
      // Payload 2: 32 mask prototype weights per image
      // 160x160 mask prototype per image
      // Mask => (mask weights * mask prototypes) > threshold

      auto keep = non_max_suppression(
          p1); // [bc, max_num_detections, 6 + 32]
               // 6 = 4(box x1,y1,x2,y2)+1(conf)+1(class) +32(mask weights)

      auto const batch_size = keep.size(0);
      auto max_num_detections = keep.size(1);

      auto p2_flat = p2.view({batch_size, 32, -1}); // [bc, 32, 25600]
      auto mask_weights = // [bs, max_num_detections, 32]
          keep.index({Slice(), Slice(), Slice(6, None)});

      auto masks = torch::bmm(mask_weights, p2_flat); // [bs,max,160 * 160]
      masks = masks.view({batch_size, max_num_detections, 160, 160}); // reshape
      masks = masks.sigmoid(); // [bs, max, 160, 160] binary

      // Create label images
      std::vector<cv::Mat> labels(batch_size);
      for (size_t i = 0; i < static_cast<size_t>(batch_size); ++i) {

        labels[i] = cv::Mat::zeros(640, 640, CV_32S);

        for (size_t j = 0; j < static_cast<size_t>(max_num_detections); ++j) {

          auto conf = keep[i][j][4].item<float>(); // [bs, max_num_detections]

          // Skip low confidence detections
          if (conf < 0.5)
            continue; // break;

          auto id = keep[i][j][5].item<int>(); // [bs, max_num_detections]
          auto box =
              keep[i][j].index({Slice(None, 4)}); // [bs, max_num_detections,4]

          cv::Mat mask(160, 160, CV_32F, masks[i][j].cpu().data_ptr<float>());
          cv::resize(mask, mask, cv::Size(640, 640), 0, 0, cv::INTER_NEAREST);

          int x1 = std::max(0, box[0].item<int>());
          int y1 = std::max(0, box[1].item<int>());
          int x2 = std::min(labels[i].cols, box[2].item<int>());
          int y2 = std::min(labels[i].rows, box[3].item<int>());

          int width = std::max(0, x2 - x1);
          int height = std::max(0, y2 - y1);

          if (width == 0 || height == 0)
            continue; // skip invalid boxes

          cv::Rect rect(x1, y1, width, height);
          labels[i](rect).setTo(id + 1, mask(rect) > 0.5);
        }
      }

      if (false) { // Visualize results
        for (size_t i = 0; i < static_cast<size_t>(batch_size); ++i) {
          cv::Mat img =
              cv::Mat(640, 640, CV_32FC3,
                      data[i]
                          .cpu()
                          .permute({1, 2, 0} /*(C, H, W) -> (H, W, C)*/)
                          .contiguous()
                          .data_ptr<float>());
          img.convertTo(img, CV_8UC3, 255.0);
          cv::imshow("Image", drawPred(img, keep[i], masks[i]));
          cv::waitKey(0);
        }
      }

      // Save output
      dataset.save(labels, node_ids);

      ++logger;
    }
  }

  return 0;
}

} // namespace ReUseX::vision

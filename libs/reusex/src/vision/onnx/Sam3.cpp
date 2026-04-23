// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/common/create_object.hpp"
#include "vision/onnx/Sam3.hpp"
#include "vision/onnx/Sam3Data.hpp"
#include "vision/osd/osd.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <span>
#include <string>

namespace reusex::vision::onnx {

namespace object = ::reusex::vision::common::object;

namespace {

/// @brief Load a file into a string (for tokenizer JSON blob).
std::string load_file_bytes(const std::filesystem::path &path) {
  std::ifstream fs(path, std::ios::in | std::ios::binary);
  if (fs.fail())
    throw std::runtime_error("Failed to open file: " + path.string());

  std::string data;
  fs.seekg(0, std::ios::end);
  auto size = static_cast<std::size_t>(fs.tellg());
  fs.seekg(0, std::ios::beg);
  data.resize(size);
  fs.read(data.data(), static_cast<std::streamsize>(size));
  return data;
}

/// @brief Compute aspect-ratio-preserving resize parameters.
///
/// Matches the TensorRT ResizeMatrix behaviour: uniform scale.
///
/// @param src_w Source image width.
/// @param src_h Source image height.
/// @param dst_w Target canvas width.
/// @param dst_h Target canvas height.
/// @return Tuple of (scale, pad_left, pad_top) in pixels.
std::tuple<float, int, int> compute_resize_params(int src_w, int src_h,
                                                   int dst_w, int dst_h) {
  float scale_x = static_cast<float>(dst_w) / static_cast<float>(src_w);
  float scale_y = static_cast<float>(dst_h) / static_cast<float>(src_h);
  float scale = std::min(scale_x, scale_y);

  int new_w = static_cast<int>(std::round(src_w * scale));
  int new_h = static_cast<int>(std::round(src_h * scale));
  int pad_left = (dst_w - new_w) / 2;
  int pad_top = (dst_h - new_h) / 2;

  return {scale, pad_left, pad_top};
}

/// @brief Build the 2x3 affine matrix that maps from mask coordinates
/// back to the original image.
///
/// mask_coords -> input_coords -> original_coords
cv::Mat build_mask_to_image_affine(int src_w, int src_h, int input_size,
                                   int mask_w, int mask_h) {
  auto [scale, pad_left, pad_top] =
      compute_resize_params(src_w, src_h, input_size, input_size);

  // mask -> input (the mask covers the full input_size canvas)
  float m2i_sx = static_cast<float>(input_size) / static_cast<float>(mask_w);
  float m2i_sy = static_cast<float>(input_size) / static_cast<float>(mask_h);

  // input -> original (reverse of the resize+pad)
  float i2o_sx = 1.0f / scale;
  float i2o_sy = 1.0f / scale;
  float i2o_tx = -static_cast<float>(pad_left) / scale;
  float i2o_ty = -static_cast<float>(pad_top) / scale;

  // Combined: mask -> original
  cv::Mat M = (cv::Mat_<float>(2, 3) << i2o_sx * m2i_sx, 0.0f, i2o_tx, 0.0f,
               i2o_sy * m2i_sy, i2o_ty);
  return M;
}

/// @brief Build tokenizer input arrays matching TensorRT convention.
///
/// Format: [START(49406), tok1, tok2, ..., PAD(49407), PAD, ...]
/// Total length = 32.
std::pair<std::array<int64_t, 32>, std::array<int64_t, 32>>
make_ids(const std::vector<int32_t> &ids) {
  constexpr int64_t kStartToken = 49406;
  constexpr int64_t kEndToken = 49407;
  constexpr size_t kMaxTokens = 31; // 32 total - 1 for start token

  std::array<int64_t, 32> ids_arr{};
  std::array<int64_t, 32> mask_arr{};

  ids_arr.fill(kEndToken);
  mask_arr.fill(0);

  if (ids.empty())
    return {ids_arr, mask_arr};

  ids_arr[0] = kStartToken;
  mask_arr[0] = 1;

  const size_t count = std::min(ids.size(), kMaxTokens);

  for (size_t i = 0; i < count; ++i) {
    ids_arr[1 + i] = ids[i];
    mask_arr[1 + i] = 1;
  }

  return {ids_arr, mask_arr};
}

} // anonymous namespace

// --- Constructor ---

ONNXSam3::ONNXSam3(const std::filesystem::path &model_dir, bool use_cuda)
    : env_(ORT_LOGGING_LEVEL_WARNING, "ONNXSam3"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,
                                              OrtMemTypeDefault)) {
  // 0 = use all available cores
  session_options_.SetIntraOpNumThreads(0);
  session_options_.SetInterOpNumThreads(0);
  session_options_.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);

  // Attempt CUDA execution provider if requested
  if (use_cuda) {
    try {
      OrtCUDAProviderOptions cuda_options;
      cuda_options.device_id = 0;
      session_options_.AppendExecutionProvider_CUDA(cuda_options);
      reusex::info("ONNX Runtime using CUDA execution provider");
    } catch (const Ort::Exception &e) {
      reusex::warn("CUDA EP not available, falling back to CPU: {}",
                         e.what());
    }
  } else {
    reusex::info("ONNX Runtime using CPU execution provider");
  }

  // Load sub-models
  auto load_session = [&](const std::string &name)
      -> std::unique_ptr<Ort::Session> {
    auto path = model_dir / (name + ".onnx");
    reusex::debug("Loading ONNX model {} from {}", name, path.string());
    auto session =
        std::make_unique<Ort::Session>(env_, path.c_str(), session_options_);
    reusex::debug("Loaded {} successfully", name);
    return session;
  };

  vision_encoder_ = load_session("vision-encoder");
  text_encoder_ = load_session("text-encoder");
  decoder_ = load_session("decoder");

  // Load tokenizer
  auto tokenizer_path = model_dir / "tokenizer.json";
  auto blob = load_file_bytes(tokenizer_path);
  reusex::debug("Tokenizer blob loaded, {} bytes", blob.size());
  tokenizer_ = tokenizers::Tokenizer::FromBlobJSON(blob);
  reusex::debug("Tokenizer initialized, vocab size: {}",
                       tokenizer_->GetVocabSize());

  reusex::info("Loaded ONNX SAM3 model from {}", model_dir);
}

std::unique_ptr<ONNXSam3>
ONNXSam3::create(const std::filesystem::path &model_dir, bool use_cuda) {
  return std::make_unique<ONNXSam3>(model_dir, use_cuda);
}

// --- Preprocessing ---

std::pair<std::vector<float>, cv::Mat>
ONNXSam3::preprocess(const cv::Mat &image) const {
  int src_h = image.rows;
  int src_w = image.cols;

  auto [scale, pad_left, pad_top] =
      compute_resize_params(src_w, src_h, input_size_, input_size_);

  int new_w = static_cast<int>(std::round(src_w * scale));
  int new_h = static_cast<int>(std::round(src_h * scale));

  // Resize
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  // Pad to input_size_ x input_size_ with value 114
  cv::Mat padded(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(pad_left, pad_top, new_w, new_h)));

  // Convert BGR -> RGB
  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

  // Normalize to [-1, 1]: (x / 127.5) - 1.0, and convert to CHW float buffer
  int pixels = input_size_ * input_size_;
  std::vector<float> chw(3 * pixels);

  const uint8_t *src_ptr = rgb.data;
  float *r_ptr = chw.data();
  float *g_ptr = chw.data() + pixels;
  float *b_ptr = chw.data() + 2 * pixels;

  for (int i = 0; i < pixels; ++i) {
    r_ptr[i] = static_cast<float>(src_ptr[i * 3 + 0]) / 127.5f - 1.0f;
    g_ptr[i] = static_cast<float>(src_ptr[i * 3 + 1]) / 127.5f - 1.0f;
    b_ptr[i] = static_cast<float>(src_ptr[i * 3 + 2]) / 127.5f - 1.0f;
  }

  // Build mask -> original affine
  cv::Mat inv_affine =
      build_mask_to_image_affine(src_w, src_h, input_size_, mask_width_,
                                 mask_height_);

  return {std::move(chw), inv_affine};
}

// --- Tokenization ---

std::pair<std::array<int64_t, 32>, std::array<int64_t, 32>>
ONNXSam3::tokenize(const std::string &text) {
  auto it = text_cache_.find(text);
  if (it != text_cache_.end())
    return it->second;

  auto raw_ids = tokenizer_->Encode(text);
  auto result = make_ids(raw_ids);

  text_cache_[text] = result;
  return result;
}

// --- Single-image inference ---

cv::Mat ONNXSam3::infer_single(const ONNXSam3Data &sam3_data) {
  const cv::Mat &image = sam3_data.image;
  const auto &prompts = sam3_data.prompts;
  float conf_threshold = sam3_data.confidence_threshold;

  // Label image: CV_32S, -1 for background
  cv::Mat label_image(image.rows, image.cols, CV_32S, cv::Scalar(-1));

  if (image.empty() || prompts.empty())
    return label_image;

  // 1. Preprocess image
  auto [image_chw, inv_affine] = preprocess(image);

  // 2. Vision encoder
  std::array<int64_t, 4> vision_input_shape = {1, 3, input_size_, input_size_};
  Ort::Value vision_input = Ort::Value::CreateTensor<float>(
      memory_info_, image_chw.data(), image_chw.size(),
      vision_input_shape.data(), vision_input_shape.size());

  const char *vision_input_names[] = {"images"};
  const char *vision_output_names[] = {"fpn_feat_0", "fpn_feat_1", "fpn_feat_2",
                                       "fpn_pos_2"};

  auto vision_outputs = vision_encoder_->Run(
      Ort::RunOptions{nullptr}, vision_input_names, &vision_input, 1,
      vision_output_names, 4);

  // Extract FPN feature shapes and data pointers (reused across prompts)
  auto get_tensor_info = [](const Ort::Value &val) {
    auto info = val.GetTensorTypeAndShapeInfo();
    return info.GetShape();
  };

  auto fpn0_shape = get_tensor_info(vision_outputs[0]);
  auto fpn1_shape = get_tensor_info(vision_outputs[1]);
  auto fpn2_shape = get_tensor_info(vision_outputs[2]);
  auto pos2_shape = get_tensor_info(vision_outputs[3]);

  float *fpn0_data = vision_outputs[0].GetTensorMutableData<float>();
  float *fpn1_data = vision_outputs[1].GetTensorMutableData<float>();
  float *fpn2_data = vision_outputs[2].GetTensorMutableData<float>();
  float *pos2_data = vision_outputs[3].GetTensorMutableData<float>();

  size_t fpn0_size = 1;
  for (auto d : fpn0_shape)
    fpn0_size *= d;
  size_t fpn1_size = 1;
  for (auto d : fpn1_shape)
    fpn1_size *= d;
  size_t fpn2_size = 1;
  for (auto d : fpn2_shape)
    fpn2_size *= d;
  size_t pos2_size = 1;
  for (auto d : pos2_shape)
    pos2_size *= d;

  // 3. Process each prompt: text encode -> decode -> postprocess
  InferResult all_detections;

  for (size_t prompt_idx = 0; prompt_idx < prompts.size(); ++prompt_idx) {
    const auto &prompt = prompts[prompt_idx];

    // Text encode
    auto [input_ids, attention_mask] = tokenize(prompt.text);

    std::array<int64_t, 2> text_shape = {1, 32};

    Ort::Value text_inputs[] = {
        Ort::Value::CreateTensor<int64_t>(memory_info_, input_ids.data(),
                                          input_ids.size(), text_shape.data(),
                                          text_shape.size()),
        Ort::Value::CreateTensor<int64_t>(
            memory_info_, attention_mask.data(), attention_mask.size(),
            text_shape.data(), text_shape.size()),
    };

    const char *text_input_names[] = {"input_ids", "attention_mask"};
    const char *text_output_names[] = {"text_features", "text_mask"};

    auto text_outputs = text_encoder_->Run(
        Ort::RunOptions{nullptr}, text_input_names, text_inputs, 2,
        text_output_names, 2);

    // Get text feature data (prompt_features = text_features for text-only)
    auto text_feat_shape = get_tensor_info(text_outputs[0]);
    auto text_mask_shape = get_tensor_info(text_outputs[1]);

    float *text_feat_data = text_outputs[0].GetTensorMutableData<float>();
    bool *text_mask_data = text_outputs[1].GetTensorMutableData<bool>();

    size_t text_feat_size = 1;
    for (auto d : text_feat_shape)
      text_feat_size *= d;
    size_t text_mask_size = 1;
    for (auto d : text_mask_shape)
      text_mask_size *= d;

    // Decode: create input tensors pointing to vision features (zero-copy)
    Ort::Value decoder_inputs[] = {
        Ort::Value::CreateTensor<float>(memory_info_, fpn0_data, fpn0_size,
                                        fpn0_shape.data(), fpn0_shape.size()),
        Ort::Value::CreateTensor<float>(memory_info_, fpn1_data, fpn1_size,
                                        fpn1_shape.data(), fpn1_shape.size()),
        Ort::Value::CreateTensor<float>(memory_info_, fpn2_data, fpn2_size,
                                        fpn2_shape.data(), fpn2_shape.size()),
        Ort::Value::CreateTensor<float>(memory_info_, pos2_data, pos2_size,
                                        pos2_shape.data(), pos2_shape.size()),
        Ort::Value::CreateTensor<float>(
            memory_info_, text_feat_data, text_feat_size,
            text_feat_shape.data(), text_feat_shape.size()),
        Ort::Value::CreateTensor<bool>(memory_info_, text_mask_data,
                                       text_mask_size, text_mask_shape.data(),
                                       text_mask_shape.size()),
    };

    const char *decoder_input_names[] = {"fpn_feat_0", "fpn_feat_1",
                                         "fpn_feat_2", "fpn_pos_2",
                                         "prompt_features", "prompt_mask"};
    const char *decoder_output_names[] = {"pred_masks", "pred_boxes",
                                          "pred_logits", "presence_logits"};

    auto dec_outputs = decoder_->Run(
        Ort::RunOptions{nullptr}, decoder_input_names, decoder_inputs, 6,
        decoder_output_names, 4);

    // Postprocess
    float *pred_masks = dec_outputs[0].GetTensorMutableData<float>();
    float *pred_boxes = dec_outputs[1].GetTensorMutableData<float>();
    float *pred_logits = dec_outputs[2].GetTensorMutableData<float>();
    float *presence_logits = dec_outputs[3].GetTensorMutableData<float>();

    float presence_score = 1.0f / (1.0f + std::exp(-presence_logits[0]));

    // Precompute box transform: normalized [0,1] input-canvas coords → image
    // coords. The inv_affine maps mask pixels to image pixels; since boxes are
    // normalized to input canvas (same space as mask), multiply by mask dims
    // first, then apply the affine.
    float box_sx = inv_affine.at<float>(0, 0) * static_cast<float>(mask_width_);
    float box_sy = inv_affine.at<float>(1, 1) * static_cast<float>(mask_height_);
    float box_tx = inv_affine.at<float>(0, 2);
    float box_ty = inv_affine.at<float>(1, 2);

    // Apply sigmoid to logits and filter by confidence
    for (int q = 0; q < num_queries_; ++q) {
      float logit = pred_logits[q];
      float score =
          (1.0f / (1.0f + std::exp(-logit))) * presence_score; // sigmoid
      if (score < conf_threshold)
        continue;

      // Extract box: pred_boxes are [x1, y1, x2, y2] normalized to input
      // canvas. Transform through mask→image affine to undo letterbox.
      float left = box_sx * pred_boxes[q * 4 + 0] + box_tx;
      float top = box_sy * pred_boxes[q * 4 + 1] + box_ty;
      float right = box_sx * pred_boxes[q * 4 + 2] + box_tx;
      float bottom = box_sy * pred_boxes[q * 4 + 3] + box_ty;

      // Clip to image bounds
      left = std::max(0.0f, left);
      top = std::max(0.0f, top);
      right = std::min(static_cast<float>(image.cols), right);
      bottom = std::min(static_cast<float>(image.rows), bottom);

      int box_w = static_cast<int>(right - left);
      int box_h = static_cast<int>(bottom - top);
      if (box_w <= 0 || box_h <= 0)
        continue;

      // Warp mask from mask space to original image space
      float *mask_data = pred_masks + q * mask_height_ * mask_width_;
      cv::Mat mask_float(mask_height_, mask_width_, CV_32F, mask_data);

      cv::Mat warped_mask;
      cv::warpAffine(mask_float, warped_mask, inv_affine,
                     cv::Size(image.cols, image.rows), cv::INTER_LINEAR,
                     cv::BORDER_CONSTANT, cv::Scalar(0));

      // Threshold mask at 0 (sigmoid was already applied in model)
      cv::Mat bin_mask;
      cv::threshold(warped_mask, bin_mask, 0.0, 255, cv::THRESH_BINARY);
      bin_mask.convertTo(bin_mask, CV_8U);

      // Crop to bounding box
      cv::Rect roi(static_cast<int>(left), static_cast<int>(top), box_w,
                   box_h);
      roi &= cv::Rect(0, 0, image.cols, image.rows);
      if (roi.area() <= 0)
        continue;

      cv::Mat cropped_mask = bin_mask(roi).clone();

      int label_id = static_cast<int>(prompt_idx);

      reusex::trace(
          "Detection: prompt='{}' ({}), score={:.3f}, box=[{:.0f}, "
          "{:.0f}, {:.0f}, {:.0f}]",
          prompt.text, label_id, score, left, top, right, bottom);

      all_detections.push_back(object::create_segmentation_box(
          left, top, right, bottom, cropped_mask, score, label_id,
          prompt.text));
    }
  }

  reusex::debug("Total detections for image: {}", all_detections.size());

  // Assemble final label image
  osd::make_labled_image(label_image, all_detections);

  return label_image;
}

// --- Forward ---

std::vector<IDataset::Pair>
ONNXSam3::forward(const std::span<IDataset::Pair> &input) {
  if (input.empty())
    return {};

  reusex::debug("ONNXSam3 forward pass with {} inputs", input.size());

  std::vector<IDataset::Pair> results(input.size());

  for (size_t i = 0; i < input.size(); ++i) {
    auto *sam3_data =
        dynamic_cast<const ONNXSam3Data *>(input[i].first.get());
    if (!sam3_data) {
      reusex::error(
          "ONNXSam3::forward: input {} is not ONNXSam3Data", i);
      throw std::runtime_error(
          "ONNXSam3::forward: input is not ONNXSam3Data");
    }

    auto res_ptr = std::make_unique<ONNXSam3Data>();
    res_ptr->image = infer_single(*sam3_data);

    results[i].first = std::move(res_ptr);
    results[i].second = input[i].second;
  }

  reusex::debug("ONNXSam3 forward pass completed");
  return results;
}

} // namespace reusex::vision::onnx

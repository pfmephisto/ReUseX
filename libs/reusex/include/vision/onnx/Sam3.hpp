// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IModel.hpp"
#include "reusex/vision/common/object.hpp"
#include "reusex/vision/onnx/Sam3Data.hpp"

#include <onnxruntime_cxx_api.h>
#include <tokenizers_cpp.h>

#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace ReUseX::vision::onnx {

/// @brief SAM3 segmentation model using ONNX Runtime for CPU/GPU inference.
///
/// Loads three ONNX sub-models (vision encoder, text encoder, decoder)
/// from a directory and implements the IModel interface for use with the
/// backend-agnostic annotation pipeline.
///
/// Optionally uses CUDA execution provider when available, with automatic
/// fallback to CPU.
class ONNXSam3 : public IModel {
    public:
  /// @brief Construct and load a SAM3 model from a directory.
  /// @param model_dir Directory containing vision-encoder.onnx, text-encoder.onnx,
  ///                  decoder.onnx, and tokenizer.json.
  /// @param use_cuda Whether to attempt CUDA execution provider.
  explicit ONNXSam3(const std::filesystem::path &model_dir,
                    bool use_cuda = false);

  /// @brief Factory method for creating an ONNXSam3 instance.
  /// @param model_dir Directory with ONNX model files.
  /// @param use_cuda Whether to use CUDA (defaults to false).
  /// @return Unique pointer to the created model.
  static std::unique_ptr<ONNXSam3>
  create(const std::filesystem::path &model_dir, bool use_cuda = false);

  /// @brief Run SAM3 inference on a batch of images.
  ///
  /// Each input Pair must contain an ONNXSam3Data with a raw image and
  /// text prompts. Output Pairs contain the same data type with the image
  /// field set to the label image (CV_32S, -1 for background).
  ///
  /// @param input Span of (data, index) pairs from the dataset.
  /// @return Vector of (data, index) pairs with label images.
  std::vector<IDataset::Pair>
  forward(const std::span<IDataset::Pair> &input) override;

    private:
  using InferResult = common::object::DetectionBoxArray;

  /// @brief Preprocess an image for the vision encoder.
  ///
  /// Resizes with aspect-ratio preservation to input_size_ x input_size_,
  /// pads with value 114, normalizes to [-1, 1], and converts BGR->RGB.
  /// Returns the CHW float buffer and the mask-to-original affine matrix.
  ///
  /// @param image Input BGR image.
  /// @return Tuple of (CHW float vector, inverse affine 2x3).
  std::pair<std::vector<float>, cv::Mat> preprocess(const cv::Mat &image) const;

  /// @brief Tokenize a text prompt and return padded arrays.
  /// @param text The prompt text string.
  /// @return Pair of (input_ids [32], attention_mask [32]) as int64 arrays.
  std::pair<std::array<int64_t, 32>, std::array<int64_t, 32>>
  tokenize(const std::string &text);

  /// @brief Run the full pipeline for one image with all its prompts.
  /// @param sam3_data Input data with image and prompts.
  /// @return Label image (CV_32S, -1 for background).
  cv::Mat infer_single(const ONNXSam3Data &sam3_data);

  // ONNX Runtime state
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> vision_encoder_;
  std::unique_ptr<Ort::Session> text_encoder_;
  std::unique_ptr<Ort::Session> decoder_;

  // Tokenizer
  std::unique_ptr<tokenizers::Tokenizer> tokenizer_;

  // Model parameters
  int input_size_ = 1008;
  int num_queries_ = 200;
  int mask_height_ = 288;
  int mask_width_ = 288;

  // Tokenizer cache: text -> (input_ids, attention_mask)
  std::unordered_map<std::string,
                     std::pair<std::array<int64_t, 32>, std::array<int64_t, 32>>>
      text_cache_;

  // Memory allocator
  Ort::MemoryInfo memory_info_;
};

} // namespace ReUseX::vision::onnx

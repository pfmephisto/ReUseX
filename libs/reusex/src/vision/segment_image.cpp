// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/segment_image.hpp"

#include "core/logging.hpp"
#include "vision/IDataset.hpp"

// Include backend data types conditionally. These live inside the vision
// library where the REUSEX_USE_* defines are set (PRIVATE on reusex_vision).
// The data headers only define plain C++ structs — no TRT/ONNX runtime headers
// are pulled in by this path.
#ifdef REUSEX_USE_TENSORRT
#include "vision/tensor_rt/Data.hpp"
#endif
#ifdef REUSEX_USE_ONNX
#include "vision/onnx/Sam3Data.hpp"
#endif

#include <opencv2/core.hpp>

#include <exception>
#include <memory>
#include <span>
#include <vector>

namespace reusex::vision {

cv::Mat segment_image(IModel &model, const cv::Mat &image_bgr,
                      const std::vector<Sam3Prompt> &prompts,
                      float confidence) {
  if (image_bgr.empty()) {
    reusex::warn("segment_image: called with an empty image");
    return {};
  }

  // --- TensorRT path (primary) -------------------------------------------
  //
  // Dispatch relies on a backend asymmetry: ONNX forward() throws when handed
  // TensorRTData (wrong IData type); TRT forward() silently returns empty pairs
  // on wrong input. The try/catch below catches the ONNX throw and falls
  // through to the ONNX path. If TRT returns empty (model produced no
  // detections), the debug log fires and we also fall through — this is
  // benign in a dual-backend build (ONNX gets a second chance on the same
  // image with the correct data type). Future backends must either throw or
  // return empty for incompatible input to keep this working. (#409 follow-up:
  // replace with explicit model-type dispatch via BackendFactory.)
#ifdef REUSEX_USE_TENSORRT
  {
    using tensor_rt::Sam3PromptUnit;
    using tensor_rt::TensorRTData;

    auto data = std::make_unique<TensorRTData>();
    data->image = image_bgr.clone();
    data->confidence_threshold = confidence;

    if (!prompts.empty()) {
      data->prompts.clear();
      for (const auto &p : prompts) {
        Sam3PromptUnit unit(p.text, {}, p.confidence);
        for (const auto &[lbl, box] : p.boxes)
          unit.boxes.emplace_back(lbl, box);
        data->prompts.push_back(std::move(unit));
      }
    }

    std::vector<IDataset::Pair> batch;
    batch.emplace_back(std::move(data), static_cast<std::size_t>(0));
    std::span<IDataset::Pair> span(batch);

    try {
      auto results = model.forward(span);
      if (!results.empty()) {
        auto *trt = dynamic_cast<TensorRTData *>(results[0].first.get());
        if (trt && !trt->image.empty()) {
          reusex::debug("segment_image: TRT produced {}x{} label map",
                        trt->image.cols, trt->image.rows);
          return trt->image;
        }
      }
      reusex::debug("segment_image: TRT forward returned empty result");
    } catch (const std::exception &e) {
      reusex::debug(
          "segment_image: TRT path threw ({}); falling through to ONNX path",
          e.what());
    }
  }
#endif

  // --- ONNX Runtime path (fallback) --------------------------------------
#ifdef REUSEX_USE_ONNX
  {
    using onnx::ONNXSam3Data;
    using onnx::Sam3PromptUnit;

    auto data = std::make_unique<ONNXSam3Data>();
    data->image = image_bgr.clone();
    data->confidence_threshold = confidence;

    if (!prompts.empty()) {
      data->prompts.clear();
      for (const auto &p : prompts) {
        // ONNXSam3Data has no per-prompt confidence field; the global threshold
        // applies to all prompts. Log if the caller set a per-prompt override
        // so the silent drop is discoverable.
        if (p.confidence >= 0.0f)
          reusex::debug("segment_image/ONNX: per-prompt confidence {:.2f} for "
                        "'{}' is not supported by the ONNX backend; "
                        "global threshold {:.2f} applies",
                        p.confidence, p.text, confidence);
        Sam3PromptUnit unit(p.text);
        for (const auto &[lbl, box] : p.boxes)
          unit.boxes.emplace_back(lbl, box);
        data->prompts.push_back(std::move(unit));
      }
    }

    std::vector<IDataset::Pair> batch;
    batch.emplace_back(std::move(data), static_cast<std::size_t>(0));
    std::span<IDataset::Pair> span(batch);

    auto results = model.forward(span);
    if (!results.empty()) {
      auto *onnx_data = dynamic_cast<ONNXSam3Data *>(results[0].first.get());
      if (onnx_data && !onnx_data->image.empty()) {
        reusex::debug("segment_image: ONNX produced {}x{} label map",
                      onnx_data->image.cols, onnx_data->image.rows);
        return onnx_data->image;
      }
    }
    reusex::debug("segment_image: ONNX forward returned empty result");
  }
#endif

  reusex::warn("segment_image: model produced no label map "
               "(no backend compiled or no detections above threshold)");
  return cv::Mat(image_bgr.size(), CV_32S, cv::Scalar(-1));
}

} // namespace reusex::vision

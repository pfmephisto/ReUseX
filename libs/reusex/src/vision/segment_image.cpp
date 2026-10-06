// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/segment_image.hpp"

#include "core/logging.hpp"
#include "vision/IDataset.hpp"
#include "vision/sam3_geometry.hpp"

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
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <memory>
#include <span>
#include <vector>

namespace reusex::vision {

namespace {

bool has_boxes(const std::vector<Sam3Prompt> &prompts) {
  return std::any_of(prompts.begin(), prompts.end(), [](const Sam3Prompt &p) {
    return !p.boxes.empty() || !p.points.empty();
  });
}

/// @p prompts with each click point turned into its smallest exemplar box, so
/// a backend that ignored the geometry clips a point prompt to the click's
/// neighbourhood.
std::vector<Sam3Prompt> points_as_boxes(const std::vector<Sam3Prompt> &prompts,
                                        int width, int height) {
  std::vector<Sam3Prompt> out = prompts;
  for (auto &p : out) {
    for (const auto &pt : p.points)
      p.boxes.emplace_back(
          "pos", sam3_point_exemplar_box(pt, width, height,
                                         sam3_point_exemplar_fracs().front()));
    p.points.clear();
  }
  return out;
}

/// Clip boxed prompts when the backend ignored the boxes, warning once per
/// process (every request would repeat the same news).
void clip_ignored_boxes(cv::Mat &labels, const std::vector<Sam3Prompt> &prompts,
                        const char *backend) {
  if (labels.empty() || !has_boxes(prompts))
    return;
  static std::atomic<bool> warned{false};
  if (!warned.exchange(true))
    reusex::warn("segment_image: the {} backend has no SAM3 geometry encoder, "
                 "so prompt boxes are not fed to the model; each boxed "
                 "prompt's detections are clipped to its boxes instead",
                 backend);
  const auto cleared = clip_labels_to_prompt_boxes(
      labels, points_as_boxes(prompts, labels.cols, labels.rows));
  reusex::debug("segment_image: clipped {} pixel(s) outside prompt boxes",
                cleared);
}

} // namespace

std::size_t
clip_labels_to_prompt_boxes(cv::Mat &labels,
                            const std::vector<Sam3Prompt> &prompts) {
  if (labels.empty() || labels.type() != CV_32SC1)
    return 0;
  const cv::Rect image(0, 0, labels.cols, labels.rows);
  auto to_rect = [&](const std::array<float, 4> &b) {
    const int x1 = static_cast<int>(std::floor(std::min(b[0], b[2])));
    const int y1 = static_cast<int>(std::floor(std::min(b[1], b[3])));
    const int x2 = static_cast<int>(std::ceil(std::max(b[0], b[2])));
    const int y2 = static_cast<int>(std::ceil(std::max(b[1], b[3])));
    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)) & image;
  };
  std::size_t cleared = 0;
  for (std::size_t k = 0; k < prompts.size(); ++k) {
    const auto &boxes = prompts[k].boxes;
    if (boxes.empty())
      continue;
    cv::Mat keep(labels.size(), CV_8UC1, cv::Scalar(0));
    bool any_pos = false;
    for (const auto &[polarity, box] : boxes)
      if (polarity == "pos") {
        any_pos = true;
        keep(to_rect(box)).setTo(255);
      }
    if (!any_pos)
      keep.setTo(255);
    for (const auto &[polarity, box] : boxes)
      if (polarity == "neg")
        keep(to_rect(box)).setTo(0);
    const cv::Mat drop = (labels == static_cast<int>(k)) & (keep == 0);
    cleared += static_cast<std::size_t>(cv::countNonZero(drop));
    labels.setTo(-1, drop);
  }
  return cleared;
}

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
    // Label k = prompt k (what our callers map labels with), not the
    // model's per-text cache id that `rux create annotate` relies on.
    data->label_by_prompt_index = true;
    // A box picks the object it covers, not every look-alike in the image.
    data->select_box_instances = true;

    if (!prompts.empty()) {
      data->prompts.clear();
      for (const auto &p : prompts) {
        Sam3PromptUnit unit(p.text, {}, p.confidence);
        for (const auto &[lbl, box] : p.boxes)
          unit.boxes.emplace_back(lbl, box);
        unit.points = p.points;
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
          if (!trt->geometry_prompts_used)
            clip_ignored_boxes(trt->image, prompts,
                               "TensorRT (no geometry-encoder.engine)");
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
        // The ONNX path runs text encoder + decoder only; boxes never reach
        // the model.
        clip_ignored_boxes(onnx_data->image, prompts, "ONNX Runtime");
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

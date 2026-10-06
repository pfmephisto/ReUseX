// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/tensor_rt/Sam3Type.hpp"

#include <opencv2/core/mat.hpp>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace reusex::vision::tensor_rt {

/* TensorRTData is a struct that implements the IData interface and contains a
 * cv::Mat image. This struct is used to store the image data that will be
 * processed by TensorRT.
 */
struct TensorRTData : IData {

  using Vec = std::array<int64_t, 32>;
  using Prompt = std::pair<std::shared_ptr<Vec>, std::shared_ptr<Vec>>;

  cv::Mat image;
  // Default concept prompts tuned for the ReUseX use case (building interior
  // reuse/renovation): the reusable building components a scan should catalogue
  // — structure/envelope, openings, circulation, building services, and fixed
  // fixtures. Movable furniture is intentionally minimal (it is not a
  // "building" component). Override per-run with `rux create annotate
  // --prompts` /
  // `--prompts-file` (the prompt list IS the class set for this open-vocab
  // model).
  std::vector<Sam3PromptUnit> prompts = {
      // Structure / envelope
      Sam3PromptUnit("wall"),
      Sam3PromptUnit("floor"),
      Sam3PromptUnit("ceiling"),
      Sam3PromptUnit("column"),
      Sam3PromptUnit("beam"),
      // Openings
      Sam3PromptUnit("door"),
      Sam3PromptUnit("door frame"),
      Sam3PromptUnit("window"),
      // Circulation
      Sam3PromptUnit("staircase"),
      Sam3PromptUnit("railing"),
      // Building services
      Sam3PromptUnit("radiator"),
      Sam3PromptUnit("pipe"),
      Sam3PromptUnit("duct"),
      Sam3PromptUnit("electrical outlet"),
      Sam3PromptUnit("ceiling light"),
      // Fixed fixtures
      Sam3PromptUnit("sink"),
      Sam3PromptUnit("cabinet"),
      Sam3PromptUnit("shelf"),
  };

  float confidence_threshold = 0.5f;

  /// Input: write each detection's prompt position (0..N-1 in `prompts`) as
  /// its label value instead of the model's per-text cache id. Set by the
  /// single-image callers (segment_image, segment_panorama), whose callers
  /// map label k to prompt k. Left false by the annotate dataset, whose class
  /// map relies on the cache-id numbering. See sam3_label_value().
  bool label_by_prompt_index = false;

  /// Output (on forward() results): true when the model has a geometry
  /// encoder, i.e. prompt boxes were fed to SAM3. False means boxes were
  /// ignored by the model and only the text drove the detections.
  bool geometry_prompts_used = false;

  /// Returns the plain text of the built-in default prompt list so callers
  /// can merge it with extra prompts (e.g. glass classes) without including
  /// this backend-specific header.
  static const std::vector<std::string> &default_prompt_text_list() {
    static const std::vector<std::string> texts{
        "wall",          "floor",
        "ceiling",       "column",
        "beam",          "door",
        "door frame",    "window",
        "staircase",     "railing",
        "radiator",      "pipe",
        "duct",          "electrical outlet",
        "ceiling light", "sink",
        "cabinet",       "shelf"};
    return texts;
  }
};
} // namespace reusex::vision::tensor_rt

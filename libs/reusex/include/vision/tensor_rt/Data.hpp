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
};
} // namespace reusex::vision::tensor_rt

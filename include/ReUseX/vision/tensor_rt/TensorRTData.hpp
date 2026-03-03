#pragma once
#include <ReUseX/vision/IData.hpp>
#include <opencv4/opencv2/core/mat.hpp>

#include <ReUseX/vision/tensor_rt/infer/sam3type.hpp>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace ReUseX::vision::tensor_rt {

/* TensorRTData is a struct that implements the IData interface and contains a
 * cv::Mat image. This struct is used to store the image data that will be
 * processed by TensorRT.
 */
struct TensorRTData : IData {

  using Vec = std::array<int64_t, 32>;
  using Prompt = std::pair<std::shared_ptr<Vec>, std::shared_ptr<Vec>>;

  cv::Mat image;
  std::vector<Sam3PromptUnit> prompts = {
	  
      Sam3PromptUnit("wall")
      ,Sam3PromptUnit("floor")
      /*, Sam3PromptUnit("ceiling"), Sam3PromptUnit("door"),
	Sam3PromptUnit("window"),  Sam3PromptUnit("radiator"),
	Sam3PromptUnit("lamp"),    Sam3PromptUnit("table"),
	Sam3PromptUnit("chair")
	*/};
  float confidence_threshold = 0.1f;
};
} // namespace ReUseX::vision::tensor_rt

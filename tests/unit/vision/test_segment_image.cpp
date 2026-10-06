// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for vision::segment_image (#409).
//
// CPU-only tests: Sam3Prompt construction, empty-image guard, and mock-model
// round-trips that verify prompt→data→label-map plumbing without GPU.
// Both TRT and ONNX data headers are included unconditionally (they are plain
// structs with no runtime dependency). The mock handles whichever data type
// segment_image() creates at runtime, making tests pass in TRT-only, ONNX-
// only, or dual-backend builds.
//
// Real-engine tests are tagged [gpu] and skip when no device/engines are found.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

// Public headers (no backend runtime required — just struct definitions).
#include <reusex/vision/onnx/Sam3Data.hpp>
#include <reusex/vision/sam3_prompt.hpp>
#include <reusex/vision/segment_image.hpp>
#include <reusex/vision/tensor_rt/Data.hpp>

#include <opencv2/core.hpp>

#include <memory>
#include <span>
#include <string>
#include <vector>

using namespace reusex::vision;
using reusex::vision::onnx::ONNXSam3Data;
using reusex::vision::tensor_rt::TensorRTData;

// ---------------------------------------------------------------------------
// Universal mock model — handles whichever IData subtype segment_image sends
// ---------------------------------------------------------------------------

class UniversalMockModel : public IModel {
  cv::Mat label_map_;
  mutable float recorded_confidence_ = -1.0f;

    public:
  explicit UniversalMockModel(cv::Mat lm) : label_map_(std::move(lm)) {}

  float recorded_confidence() const { return recorded_confidence_; }

  std::vector<IDataset::Pair>
  forward(const std::span<IDataset::Pair> &input) override {
    std::vector<IDataset::Pair> out;
    for (const auto &[item, idx] : input) {
      // TRT backend path.
      if (auto *trt = dynamic_cast<TensorRTData *>(item.get())) {
        recorded_confidence_ = trt->confidence_threshold;
        auto result = std::make_unique<TensorRTData>();
        result->image = label_map_.clone();
        result->prompts = trt->prompts;
        out.emplace_back(std::move(result), idx);
        continue;
      }
      // ONNX backend path.
      if (auto *onnx = dynamic_cast<ONNXSam3Data *>(item.get())) {
        recorded_confidence_ = onnx->confidence_threshold;
        auto result = std::make_unique<ONNXSam3Data>();
        result->image = label_map_.clone();
        result->prompts = onnx->prompts;
        out.emplace_back(std::move(result), idx);
        continue;
      }
    }
    return out;
  }
};

// ---------------------------------------------------------------------------
// Sam3Prompt construction (no model needed)
// ---------------------------------------------------------------------------

TEST_CASE("Sam3Prompt_TextOnly_StoresText", "[segment_image][cpu]") {
  Sam3Prompt p("wall");
  CHECK(p.text == "wall");
  CHECK(p.boxes.empty());
  CHECK(p.confidence < 0.0f); // default = use global threshold
}

TEST_CASE("Sam3Prompt_WithBoxAndConfidence_StoresAll", "[segment_image][cpu]") {
  SegmentBox box{"pos", {10.f, 20.f, 100.f, 200.f}};
  Sam3Prompt p("floor", {box}, 0.7f);
  REQUIRE(p.boxes.size() == 1);
  CHECK(p.boxes[0].first == "pos");
  CHECK(p.boxes[0].second[0] == 10.f);
  CHECK(p.boxes[0].second[2] == 100.f);
  CHECK(p.confidence == 0.7f);
}

TEST_CASE("Sam3Prompt_NegBox_LabelIsNeg", "[segment_image][cpu]") {
  SegmentBox neg{"neg", {0.f, 0.f, 5.f, 5.f}};
  Sam3Prompt p("chair", {neg});
  CHECK(p.boxes[0].first == "neg");
}

// ---------------------------------------------------------------------------
// Empty-image guard
// ---------------------------------------------------------------------------

TEST_CASE("SegmentImage_EmptyInput_ReturnsEmpty", "[segment_image][cpu]") {
  cv::Mat fixed(4, 4, CV_32S, cv::Scalar(0));
  UniversalMockModel model(fixed);

  cv::Mat result = segment_image(model, cv::Mat{});
  CHECK(result.empty());
}

// ---------------------------------------------------------------------------
// Mock-model round-trips (no GPU, no backend runtime required)
// ---------------------------------------------------------------------------

TEST_CASE("SegmentImage_MockModel_ReturnsMockLabelMap",
          "[segment_image][cpu]") {
  // 16x16 label map: label 0 in left half, label 1 in right.
  cv::Mat expected(16, 16, CV_32S, cv::Scalar(0));
  for (int r = 0; r < 16; ++r)
    for (int c = 8; c < 16; ++c)
      expected.at<int>(r, c) = 1;

  UniversalMockModel model(expected);
  cv::Mat image(16, 16, CV_8UC3, cv::Scalar(100, 100, 100));
  std::vector<Sam3Prompt> prompts{Sam3Prompt("wall"), Sam3Prompt("floor")};

  cv::Mat result = segment_image(model, image, prompts, 0.5f);

  REQUIRE(!result.empty());
  CHECK(result.type() == CV_32S);
  CHECK(result.size() == image.size());
  CHECK(result.at<int>(8, 4) == 0);  // left half
  CHECK(result.at<int>(8, 12) == 1); // right half
}

TEST_CASE("SegmentImage_EmptyPrompts_CallsForwardWithDefaultList",
          "[segment_image][cpu]") {
  // With empty prompts, the default prompt list is kept (18 for TRT, 13 for
  // ONNX). The mock returns an all-zero map; the point is that forward()
  // was called and the result is non-empty.
  cv::Mat fixed(8, 8, CV_32S, cv::Scalar(-1));
  UniversalMockModel model(fixed);
  cv::Mat image(8, 8, CV_8UC3, cv::Scalar(0));

  cv::Mat result = segment_image(model, image, {}, 0.5f);
  REQUIRE(!result.empty());
  CHECK(result.type() == CV_32S);
}

TEST_CASE("SegmentImage_ConfidenceForwardedToData", "[segment_image][cpu]") {
  cv::Mat fixed(4, 4, CV_32S, cv::Scalar(0));
  UniversalMockModel model(fixed);
  cv::Mat image(4, 4, CV_8UC3, cv::Scalar(0));

  segment_image(model, image, {Sam3Prompt("wall")}, 0.42f);

  CHECK(model.recorded_confidence() == Catch::Approx(0.42f));
}

TEST_CASE("SegmentImage_PromptsForwardedToData", "[segment_image][cpu]") {
  // Verify that non-empty prompts are forwarded, not the default list.
  struct PromptCapture : IModel {
    std::size_t num_prompts = 0;
    std::vector<IDataset::Pair>
    forward(const std::span<IDataset::Pair> &input) override {
      std::vector<IDataset::Pair> out;
      for (const auto &[item, idx] : input) {
        if (auto *trt = dynamic_cast<TensorRTData *>(item.get())) {
          num_prompts = trt->prompts.size();
          auto res = std::make_unique<TensorRTData>();
          res->image = cv::Mat(4, 4, CV_32S, cv::Scalar(0));
          out.emplace_back(std::move(res), idx);
        } else if (auto *onnx = dynamic_cast<ONNXSam3Data *>(item.get())) {
          num_prompts = onnx->prompts.size();
          auto res = std::make_unique<ONNXSam3Data>();
          res->image = cv::Mat(4, 4, CV_32S, cv::Scalar(0));
          out.emplace_back(std::move(res), idx);
        }
      }
      return out;
    }
  };

  PromptCapture model;
  cv::Mat image(4, 4, CV_8UC3, cv::Scalar(0));
  std::vector<Sam3Prompt> prompts{Sam3Prompt("wall"), Sam3Prompt("floor"),
                                  Sam3Prompt("door")};
  segment_image(model, image, prompts, 0.5f);

  CHECK(model.num_prompts == 3);
}

// ---------------------------------------------------------------------------
// Label numbering (TRT) and box clipping when the backend ignores boxes
// ---------------------------------------------------------------------------

TEST_CASE("Sam3LabelValue_PromptIndexOnlyWhenRequested",
          "[segment_image][cpu]") {
  // Single-image callers: label = the prompt's position.
  CHECK(sam3_label_value(true, 0, 5) == 0);
  CHECK(sam3_label_value(true, 3, 0) == 3);
  // Annotate dataset (flag off): the backend's per-text cache id, unchanged.
  CHECK(sam3_label_value(false, 0, 5) == 5);
  CHECK(sam3_label_value(false, 3, 0) == 0);
  // No prompt list (index -1): always the cache id.
  CHECK(sam3_label_value(true, -1, 7) == 7);
}

TEST_CASE("GeometryOnlyPromptText_IsUpstreamVisual", "[segment_image][cpu]") {
  CHECK(kGeometryOnlyPromptText == "visual");
}

TEST_CASE("ClipLabelsToPromptBoxes_PosNegAndUnboxedPrompts",
          "[segment_image][cpu]") {
  // Prompt 0 boxed (pos [2,2,6,6] minus neg [4,4,6,6]), prompt 1 unboxed,
  // prompt 2 neg-only ([0,0,2,2]). Every label covers the whole 8x8 image.
  const std::vector<Sam3Prompt> prompts{
      Sam3Prompt(
          std::string(kGeometryOnlyPromptText),
          {{"pos", {2.f, 2.f, 6.f, 6.f}}, {"neg", {4.f, 4.f, 6.f, 6.f}}}),
      Sam3Prompt("wall"), Sam3Prompt("floor", {{"neg", {0.f, 0.f, 2.f, 2.f}}})};
  for (int label = 0; label < 3; ++label) {
    INFO("label " << label);
    cv::Mat labels(8, 8, CV_32S, cv::Scalar(label));
    const auto cleared = clip_labels_to_prompt_boxes(labels, prompts);
    if (label == 0) {
      CHECK(cleared == 64u - (16u - 4u));
      CHECK(labels.at<int>(2, 2) == 0);  // inside pos
      CHECK(labels.at<int>(5, 5) == -1); // inside neg
      CHECK(labels.at<int>(0, 0) == -1); // outside pos
      CHECK(labels.at<int>(7, 7) == -1);
    } else if (label == 1) {
      CHECK(cleared == 0u);
    } else {
      CHECK(cleared == 4u);
      CHECK(labels.at<int>(1, 1) == -1);
      CHECK(labels.at<int>(3, 3) == 2);
    }
  }
}

TEST_CASE("SegmentImage_TrtInput_AsksForPromptIndexLabels",
          "[segment_image][cpu]") {
  struct FlagCapture : IModel {
    bool saw_trt = false, flag = false;
    std::vector<IDataset::Pair>
    forward(const std::span<IDataset::Pair> &input) override {
      std::vector<IDataset::Pair> out;
      for (const auto &[item, idx] : input) {
        if (auto *trt = dynamic_cast<TensorRTData *>(item.get())) {
          saw_trt = true;
          flag = trt->label_by_prompt_index;
          auto res = std::make_unique<TensorRTData>();
          res->image = cv::Mat(4, 4, CV_32S, cv::Scalar(0));
          out.emplace_back(std::move(res), idx);
        } else if (dynamic_cast<ONNXSam3Data *>(item.get())) {
          auto res = std::make_unique<ONNXSam3Data>();
          res->image = cv::Mat(4, 4, CV_32S, cv::Scalar(0));
          out.emplace_back(std::move(res), idx);
        }
      }
      return out;
    }
  };
  FlagCapture model;
  segment_image(model, cv::Mat(4, 4, CV_8UC3, cv::Scalar(0)),
                {Sam3Prompt("wall")}, 0.5f);
  if (!model.saw_trt)
    SKIP("TensorRT backend not compiled in");
  CHECK(model.flag);
  CHECK_FALSE(TensorRTData{}.label_by_prompt_index); // annotate default
}

TEST_CASE("SegmentImage_BackendWithoutGeometry_ClipsBoxedPrompts",
          "[segment_image][cpu]") {
  // The mock reports geometry_prompts_used = false (TRT default) and the ONNX
  // path never uses boxes, so either way the boxed prompt is clipped.
  UniversalMockModel model(cv::Mat(8, 8, CV_32S, cv::Scalar(0)));
  const cv::Mat image(8, 8, CV_8UC3, cv::Scalar(0));
  const cv::Mat result =
      segment_image(model, image,
                    {Sam3Prompt(std::string(kGeometryOnlyPromptText),
                                {{"pos", {0.f, 0.f, 4.f, 4.f}}})},
                    0.5f);
  CHECK(cv::countNonZero(result == 0) == 16);
  CHECK(result.at<int>(7, 7) == -1);
}

TEST_CASE("SegmentImage_TrtWithGeometry_DoesNotClip", "[segment_image][cpu]") {
  struct GeomModel : IModel {
    bool saw_trt = false;
    std::vector<IDataset::Pair>
    forward(const std::span<IDataset::Pair> &input) override {
      std::vector<IDataset::Pair> out;
      for (const auto &[item, idx] : input)
        if (dynamic_cast<TensorRTData *>(item.get())) {
          saw_trt = true;
          auto res = std::make_unique<TensorRTData>();
          res->image = cv::Mat(8, 8, CV_32S, cv::Scalar(0));
          res->geometry_prompts_used = true;
          out.emplace_back(std::move(res), idx);
        }
      return out;
    }
  };
  GeomModel model;
  const cv::Mat result = segment_image(
      model, cv::Mat(8, 8, CV_8UC3, cv::Scalar(0)),
      {Sam3Prompt("door", {{"pos", {0.f, 0.f, 4.f, 4.f}}})}, 0.5f);
  if (!model.saw_trt)
    SKIP("TensorRT backend not compiled in");
  CHECK(cv::countNonZero(result == 0) == 64);
}

// ---------------------------------------------------------------------------
// [gpu] placeholder: real-engine test (skipped without CUDA device / engines)
// ---------------------------------------------------------------------------

TEST_CASE("SegmentImage_RealEngine_ProducesCV32SLabelMap",
          "[segment_image][gpu]") {
  const char *model_env = std::getenv("REUSEX_MODEL_PATH");
  if (!model_env)
    SKIP("REUSEX_MODEL_PATH not set; skipping GPU inference test");
  SKIP("GPU inference test deferred until CI has a GPU runner (see #409)");
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/segment_frame.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/vision/model_factory.hpp>
#include <reusex/vision/sam3_prompt.hpp>
#include <reusex/vision/segment_image.hpp>

#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// Parse "pos:x1,y1,x2,y2" or "neg:x1,y1,x2,y2" into a SegmentBox.
/// Throws std::invalid_argument on malformed input.
reusex::vision::SegmentBox parse_box(const std::string &spec) {
  const auto colon = spec.find(':');
  if (colon == std::string::npos)
    throw std::invalid_argument(fmt::format("invalid --box '{}': expected "
                                            "label:x1,y1,x2,y2",
                                            spec));

  const std::string label = spec.substr(0, colon);
  if (label != "pos" && label != "neg")
    throw std::invalid_argument(
        fmt::format("--box label must be 'pos' or 'neg', got '{}'", label));

  const std::string coords = spec.substr(colon + 1);
  std::array<float, 4> box{};
  int parsed = 0;
  std::size_t pos = 0;
  for (int i = 0; i < 4; ++i) {
    std::size_t end;
    box[static_cast<std::size_t>(i)] = std::stof(coords.substr(pos), &end);
    pos += end;
    if (i < 3) {
      if (pos >= coords.size() || coords[pos] != ',')
        throw std::invalid_argument(
            fmt::format("--box '{}': expected 4 comma-separated coords", spec));
      ++pos;
    }
    ++parsed;
  }
  if (parsed != 4)
    throw std::invalid_argument(
        fmt::format("--box '{}': expected exactly 4 coordinates", spec));

  return {label, box};
}

} // namespace

void setup_subcommand_create_segment_frame(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandSegmentFrameOptions>();
  auto *sub = app.add_subcommand(
      "segment-frame",
      "Run SAM3 on one sensor frame and store the label mask (#409)");

  sub->footer(R"(
DESCRIPTION:
  Runs SAM3 segmentation on a single sensor frame from the project and writes
  the resulting CV_32S label map back as the frame's segmentation image.

  Prompts are open-vocabulary text class names. Add box hints with --box to
  guide SAM3 toward a specific region. To emulate a point-click, pass a small
  box centred on the clicked pixel (e.g. click ±8 px).

EXAMPLES:
  rux create segment-frame --net /models/sam3 --frame 42 --text wall floor
  rux create segment-frame --net /models/sam3 --frame 42 \
      --text "electrical outlet" --box pos:200,300,250,360 --confidence 0.3
  rux create segment-frame --net /models/sam3 --frame 42 --no-save

NOTES:
  - Requires a color image for the target frame (run 'rux import rtabmap' first).
  - --net must point to a SAM3 TRT engine directory or an ONNX model file.
  - Without --text the model's built-in default class list is used.
  - --no-save runs inference and reports statistics without writing to the DB.
)");

  sub->add_option("-n,--net", opt->net_path,
                  "SAM3 model path (TRT engine directory or .onnx file)")
      ->required();

  sub->add_option("-f,--frame", opt->frame_id, "Sensor frame id to segment")
      ->required()
      ->check(CLI::Range(0, 1000000));

  sub->add_option("-t,--text", opt->texts,
                  "Text class prompts (open-vocabulary; repeatable)");

  sub->add_option("-B,--box", opt->boxes,
                  "Bounding-box prompt: 'pos:x1,y1,x2,y2' or 'neg:x1,y1,x2,y2' "
                  "(repeatable; applies to the last --text prompt)")
      ->allow_extra_args(false);

  sub->add_option("--confidence", opt->confidence,
                  "Detection confidence threshold [0,1]")
      ->check(CLI::Range(0.0f, 1.0f))
      ->default_val(opt->confidence);

  sub->add_flag("-c,--cuda", opt->use_cuda, "Use CUDA for inference")
      ->default_val(opt->use_cuda);

  sub->add_flag("--no-save", "Skip writing the mask back to the project")
      ->default_function([opt]() { opt->save = false; });

  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_create_segment_frame(*opt, *global_opt));
  });
}

int run_subcommand_create_segment_frame(
    const SubcommandSegmentFrameOptions &opt, const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);

    // Load the frame's color image.
    const cv::Mat image = db.sensor_frame_image(opt.frame_id);
    if (image.empty()) {
      spdlog::error("Frame {} has no color image in project '{}'", opt.frame_id,
                    global_opt.project_db.string());
      return RuxError::INVALID_ARGUMENT;
    }

    // Build the prompt list from --text and --box options.
    // --box arguments attach to the last --text entry; prompts without
    // explicit boxes contain only the text (model ignores empty box list).
    std::vector<reusex::vision::Sam3Prompt> prompts;
    for (const auto &text : opt.texts)
      prompts.emplace_back(text);

    for (const auto &box_spec : opt.boxes) {
      if (prompts.empty())
        prompts.emplace_back("object"); // implicit label when --text is absent
      try {
        prompts.back().boxes.push_back(parse_box(box_spec));
      } catch (const std::invalid_argument &e) {
        spdlog::error("{}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    // Load model and run single-frame inference.
    spdlog::info("Loading SAM3 model from {}", opt.net_path.string());
    auto model =
        reusex::vision::create_model_from_path(opt.net_path, opt.use_cuda);

    spdlog::info("Segmenting frame {} ({}x{}) with {} prompt(s)", opt.frame_id,
                 image.cols, image.rows,
                 prompts.empty() ? 18 : static_cast<int>(prompts.size()));

    const cv::Mat label_map =
        reusex::vision::segment_image(*model, image, prompts, opt.confidence);

    if (label_map.empty()) {
      spdlog::warn("Segmentation produced no label map for frame {}",
                   opt.frame_id);
      return RuxError::SUCCESS;
    }

    const int labeled = cv::countNonZero(label_map != -1);
    spdlog::info("Segmentation complete: {} labeled pixels ({:.1f}%)", labeled,
                 100.0 * labeled / (label_map.rows * label_map.cols));

    if (opt.save) {
      db.save_segmentation_image(opt.frame_id, label_map);
      spdlog::info("Mask saved to frame {} in '{}'", opt.frame_id,
                   global_opt.project_db.string());
    } else {
      spdlog::info("--no-save: mask not written to project");
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("segment-frame failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

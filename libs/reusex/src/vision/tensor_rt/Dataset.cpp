#include "vision/tensor_rt/Dataset.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "vision/IData.hpp"
#include "vision/IDataset.hpp"
#include "vision/tensor_rt/Data.hpp"

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <range/v3/all.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

template <typename R> auto enumerate(R &&r) {
  return ranges::views::zip(ranges::views::iota(0ul, ranges::size(r)),
                            std::forward<R>(r));
}

namespace reusex::vision::tensor_rt {
namespace {

/* Split a caller-supplied prompt spec into its concept text and its optional
 * per-prompt confidence threshold. The spec is either "<concept>" or
 * "<concept>:<0..1>"; the suffix after the LAST ':' is only treated as a
 * threshold when it parses to a float in [0,1], so concept names may themselves
 * contain ':'. Returns confidence = -1 when no threshold is present, meaning
 * "fall back to the frame/global threshold".
 * @param spec One entry of the --prompts / --prompts-file list.
 * @return {concept text, confidence} with confidence < 0 when unset. */
std::pair<std::string, float> parse_prompt_spec(const std::string &spec) {
  if (auto pos = spec.rfind(':');
      pos != std::string::npos && pos + 1 < spec.size()) {
    try {
      size_t used = 0;
      float v = std::stof(spec.substr(pos + 1), &used);
      if (used == spec.size() - pos - 1 && v >= 0.0f && v <= 1.0f)
        return {spec.substr(0, pos), v};
    } catch (const std::exception &) {
      // not a threshold suffix; keep the whole string as the concept text
    }
  }
  return {spec, -1.0f};
}

/* Append `text` to `classes` unless it is already present, preserving
 * first-occurrence order. Mirrors how the models assign class ids: both
 * Sam3::forward and Sam3p1::step insert unseen prompt texts into
 * `text_input_map_` and use the map size at insertion time as the id, so ids
 * follow the deduplicated first-seen order of the prompt list. */
void push_unique(std::vector<std::string> &classes, std::string text) {
  if (std::find(classes.begin(), classes.end(), text) == classes.end())
    classes.push_back(std::move(text));
}

} // namespace

reusex::vision::IDataset::Pair
TensorRTDataset::get(const std::size_t index) const {
  reusex::trace("TensorRTDataset getting data at index {}", index);
  auto data = std::make_pair(std::make_unique<TensorRTData>(), index);
  data.first->image = image(index);
  data.first->confidence_threshold = confidence_;
  // Override the built-in default class list with caller-supplied prompts
  // (open-vocabulary: the prompt list IS the set of classes to detect).
  if (!prompts_.empty()) {
    data.first->prompts.clear();
    data.first->prompts.reserve(prompts_.size());
    for (const auto &p : prompts_) {
      // Optional per-prompt threshold: "concept:0.4".
      auto [text, conf] = parse_prompt_spec(p);
      data.first->prompts.emplace_back(std::move(text),
                                       std::vector<BoxPrompt>{}, conf);
    }
  }
  return data;
}

bool TensorRTDataset::save(const std::span<IDataset::Pair> &data) {
  reusex::info("Saving {} TensorRT dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    TensorRTData *trt_data = dynamic_cast<TensorRTData *>(item.get());
    if (!trt_data) {
      reusex::warn("TensorRTDataset::save skipping non-TensorRTData item at "
                   "index {}",
                   index);
      success = false;
      continue;
    }

    success &= save_image(index, trt_data->image);

    if (!class_map_saved_) {
      // The class map MUST be derived from the prompt list the model actually
      // saw, which is the one get() stamps onto the INPUT data — i.e. this
      // dataset's own prompts_ whenever the caller supplied --prompts /
      // --prompts-file. It cannot come from `trt_data`: that is the model's
      // freshly-constructed OUTPUT (`std::make_unique<TensorRTData>()` in
      // Sam3::forward and Sam3p1::step), so its `prompts` are always Data.hpp's
      // built-in defaults regardless of what the user asked for. Reading them
      // here recorded {0:"wall",1:"floor",...} while the label pixels carried
      // ids from the user's concepts, silently mislabelling every consumer.
      std::vector<std::string> classes;
      if (!prompts_.empty()) {
        for (const auto &p : prompts_)
          push_unique(classes, parse_prompt_spec(p).first);
      } else {
        // No caller prompts: get() left the IData's built-in default list, so
        // the defaults on the output data are what the model saw.
        for (const auto &prompt : trt_data->prompts)
          push_unique(classes, prompt.text);
      }

      auto json = fmt::format(
          "{{{}}}", fmt::join(enumerate(classes) |
                                  ranges::views::transform([](auto pair) {
                                    auto [i, name] = pair;
                                    return fmt::format("\"{}\":\"{}\"", i,
                                                       name);
                                  }),
                              ","));
      database()->log_pipeline_start("annotate_class_map", json);
      class_map_saved_ = true;
      reusex::info("Saved segmentation class map ({} classes)", classes.size());
    }
  }

  reusex::debug("Save operation completed with success={}", success);
  return success;
}
} // namespace reusex::vision::tensor_rt

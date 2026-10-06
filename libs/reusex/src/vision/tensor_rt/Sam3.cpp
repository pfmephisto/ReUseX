// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/tensor_rt/Sam3.hpp"
#include "core/logging.hpp"
#include "vision/IDataset.hpp"
#include "vision/common/create_object.hpp"
#include "vision/common/image.hpp"
#include "vision/osd/osd.hpp"
#include "vision/sam3_geometry.hpp"
#include "vision/sam3_prompt.hpp"
#include "vision/tensor_rt/Data.hpp"
#include "vision/tensor_rt/common/affine.hpp"
#include "vision/tensor_rt/common/device.hpp"
#include "vision/tensor_rt/kernels/postprocess.cuh"
#include "vision/tensor_rt/kernels/process_kernel_warp.hpp"

#include <fmt/ranges.h>

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

#include <algorithm>
#include <atomic>
#include <map>
#include <span>

using ArrayInt64 = std::array<int64_t, 32>;

std::pair<ArrayInt64, ArrayInt64> make_ids(const std::vector<int32_t> &ids) {

  constexpr int64_t kStartToken = 49406;
  constexpr int64_t kEndToken = 49407;
  constexpr size_t kMaxTokens = 31; // 32 total - 1 for start token

  ArrayInt64 ids_arr{ids.size()};
  ArrayInt64 mask_arr{ids.size()};

  ids_arr.fill(kEndToken);
  mask_arr.fill(0);

  if (ids.empty())
    return {ids_arr, mask_arr};

  ids_arr[0] = kStartToken;
  mask_arr[0] = 1;

  const size_t count = std::min(ids.size(), kMaxTokens);

  // Correct span declarations
  std::span<int64_t> ids_view(ids_arr.data() + 1, count);
  std::span<int64_t> mask_view(mask_arr.data() + 1, count);

  // Fill the subarrays
  std::copy_n(ids.begin(), count, ids_view.begin());
  std::fill_n(mask_view.begin(), count + 1, 1);

  return {ids_arr, mask_arr};
}

namespace reusex::vision::tensor_rt {

namespace object = ::reusex::vision::common::object;
namespace common_tensor = ::reusex::vision::common::tensor;

namespace {

/// Does detection @p det's mask (stored over its box) cover pixel (x, y)?
bool detection_mask_covers(const object::DetectionBox &det, float x, float y) {
  if (!det.segmentation || det.segmentation->mask.empty())
    return false;
  const cv::Mat &m = det.segmentation->mask;
  const int cx = static_cast<int>(std::floor(x - std::max(0.f, det.box.left)));
  const int cy = static_cast<int>(std::floor(y - std::max(0.f, det.box.top)));
  if (cx < 0 || cy < 0 || cx >= m.cols || cy >= m.rows)
    return false;
  return m.at<uint8_t>(cy, cx) != 0;
}

} // namespace

std::unique_ptr<TensorRTSam3>
TensorRTSam3::create(const std::filesystem::path &model_path) {
  reusex::info("Creating TensorRTSam3 from model path: {}", model_path);

  std::filesystem::path vision_encoder_path =
      model_path / "vision-encoder.engine";
  std::filesystem::path text_encoder_path = model_path / "text-encoder.engine";
  std::filesystem::path decoder_path = model_path / "decoder.engine";
  std::filesystem::path geometry_encoder_path =
      model_path / "geometry-encoder.engine";
  std::filesystem::path tokenizer_path = model_path / "tokenizer.json";

  int gpu_id = 0; // Default GPU ID, can be parameterized

  // The geometry encoder is optional: without it (or with engines from a
  // pre-v2 recipe, see load_engines) box prompts are not fed to the model.
  std::string geom_path = std::filesystem::exists(geometry_encoder_path)
                              ? geometry_encoder_path.string()
                              : std::string();
  auto instance = std::make_unique<TensorRTSam3>(
      vision_encoder_path, text_encoder_path, geom_path, decoder_path,
      tokenizer_path, gpu_id);

  if (!instance->load_engines()) {
    // reusex::error("Failed to load TensorRTSam3 engines from path: {}",
    //               model_path);
    reusex::error("Failed to load TensorRTSam3 engines with "
                  "vision_encoder_path : {}, "
                  "text_encoder_path : {}, geometry_encoder_path : {}, "
                  "decoder_path : {} ",
                  vision_encoder_path, text_encoder_path, geometry_encoder_path,
                  decoder_path);
    return nullptr;
  }
  reusex::info("TensorRTSam3 created successfully");
  return instance;
}

TensorRTSam3::TensorRTSam3(const std::string &vision_encoder_path,
                           const std::string &text_encoder_path,
                           const std::string &geometry_encoder_path,
                           const std::string &decoder_path,
                           const std::string &tokenizer_path, int gpu_id)
    : IModel(), vision_encoder_path_(vision_encoder_path),
      text_encoder_path_(text_encoder_path),
      geometry_encoder_path_(geometry_encoder_path),
      decoder_path_(decoder_path), gpu_id_(gpu_id) {
  reusex::debug("Initializing TensorRTSam3 with gpu_id={}, max_image_batch={}, "
                "max_prompt_batch={}",
                gpu_id_, max_image_batch_, max_prompt_batch_);

  // Initialize reserved Image Buffer
  original_images_buf_.resize(max_image_batch_);
  for (auto &buf : original_images_buf_) {
    buf = std::make_shared<tensor::Memory<uint8_t>>();
  }

  // Reserve size record
  original_image_sizes_.resize(max_image_batch_);

  // Note: all the current factory APIs takes in-memory blob as input.
  // This gives some flexibility on how these blobs can be read.
  auto blob = load_bytes_from_file(tokenizer_path);
  reusex::debug("Tokenizer blob loaded, size: {} bytes", blob.size());
  tokenizer_ = tokenizers::Tokenizer::FromBlobJSON(blob);
  reusex::debug("Tokenizer initialized successfully with vocab size: {}",
                tokenizer_->GetVocabSize());
}

std::vector<IDataset::Pair>
TensorRTSam3::forward(const std::span<IDataset::Pair> &input) {
  reusex::debug("TensorRTSam3 forward pass with {} inputs", input.size());

  if (input.empty()) {
    reusex::warn("Empty input provided to forward pass");
    return {};
  }
  std::vector<IDataset::Pair> results_img(input.size());
  std::vector<const TensorRTData *> tensor_inputs(input.size());

  // Cast inputs to TensorRTData
  for (size_t i = 0; i < input.size(); ++i) {
    // INFO: Maybe a visior patter should be considered
    tensor_inputs[i] = dynamic_cast<const TensorRTData *>(input[i].first.get());
    if (!tensor_inputs[i]) {
      reusex::error("Input at index {} is not of type TensorRTData. "
                    "Returning empty results.",
                    i);
      return results_img;
    }
  }

  // Add prompts to text_input_map_ wich is a cache for text inputs to avoid
  // redundatn tokenization
  for (auto &data : tensor_inputs) {
    for (auto &prompt : data->prompts) {

      // Skip if already tokenized (cache hit)
      if (text_input_map_.count(prompt.text) != 0)
        continue;

      // Add to cache (cache miss)
      auto [ids, mask] = make_ids(tokenizer_->Encode(prompt.text));
      int idx = (int)text_input_map_.size();
      text_input_map_[prompt.text] = std::make_tuple(ids, mask, idx);
      reusex::trace("Tokenized text: '{}' ({}), IDs: [{}], Mask: [{}]",
                    prompt.text, idx,
                    fmt::join(ids | ranges::views::take(5), ", "),
                    fmt::join(mask | ranges::views::take(5), ", "));
    }
  }

  constexpr bool return_mask = true;

  // void *stream = nullptr;
  cudaStream_t stream = nullptr;
  if (auto s = cudaStreamCreate(&stream); s != cudaSuccess)
    reusex::error("Failed to create CUDA stream: {}", cudaGetErrorString(s));

  AutoDevice device_guard(gpu_id_);

  const int num_images = (int)tensor_inputs.size();
  InferResultArray results(num_images);

  // The vision-encoder buffers (preprocessed images, FPN features, per-image
  // sizes/affine matrices) are all sized to max_image_batch_. To support an
  // arbitrary input batch we process images in chunks of max_image_batch_.
  // Within a chunk, image indices are chunk-local (0..chunk_images-1) — which
  // is what preprocess/encode_image/gather_vision_features/postprocess expect —
  // while results and inputs are addressed by the global image index
  // (img_chunk_start + local).
  for (int img_chunk_start = 0; img_chunk_start < num_images;
       img_chunk_start += max_image_batch_) {
    const int img_chunk_end =
        std::min(img_chunk_start + max_image_batch_, num_images);
    const int chunk_images = img_chunk_end - img_chunk_start;

    // 3. Vision Encoder — preprocess + encode this image chunk (local indices).
    for (int li = 0; li < chunk_images; ++li)
      preprocess(*tensor_inputs[img_chunk_start + li], li, stream);

    if (!encode_image(chunk_images, stream)) {
      reusex::error("Vision encoder failed for image chunk [{}, {}); skipping",
                    img_chunk_start, img_chunk_end);
      continue;
    }

    // Build this chunk's prompt list, tagged with chunk-local image indices.
    std::vector<PromptMeta> chunk_prompts;
    for (int li = 0; li < chunk_images; ++li) {
      const int gi = img_chunk_start + li;
      if (tensor_inputs[gi]->prompts.empty()) {
        chunk_prompts.push_back({li, -1, nullptr});
        continue;
      }
      for (size_t j = 0; j < tensor_inputs[gi]->prompts.size(); ++j) {
        const auto &prompt = tensor_inputs[gi]->prompts[j];
        // A click point is tried at several exemplar sizes (one decoder pass
        // each); the most confident detection under the point wins below.
        if (!prompt.points.empty() && geom_max_boxes_ > 0 &&
            tensor_inputs[gi]->select_box_instances) {
          for (const float frac : sam3_point_exemplar_fracs())
            chunk_prompts.push_back({li, (int)j, &prompt, frac});
        } else {
          chunk_prompts.push_back({li, (int)j, &prompt});
        }
        const size_t n_geom = prompt.boxes.size() + prompt.points.size();
        if (geom_max_boxes_ > 0 &&
            n_geom > static_cast<size_t>(geom_max_boxes_)) {
          static std::atomic<bool> warned{false};
          if (!warned.exchange(true))
            reusex::warn("TensorRTSam3: a prompt has {} boxes/points but the "
                         "engines accept {}; the extra ones are dropped",
                         n_geom, geom_max_boxes_);
        }
      }
    }

    // Point prompts: every pass's selected detections, finalised once all
    // passes of the chunk have run. Key: (global image, prompt index).
    std::map<std::pair<int, int>, InferResult> point_candidates;
    std::map<std::pair<int, int>, std::vector<float>> candidate_frac;

    // 4. Decoder batch loop — split this chunk's prompts by max_prompt_batch_.
    const int total_prompts = (int)chunk_prompts.size();
    for (int chunk_start = 0; chunk_start < total_prompts;
         chunk_start += max_prompt_batch_) {
      const int chunk_end =
          std::min(chunk_start + max_prompt_batch_, total_prompts);
      const int current_batch_size = chunk_end - chunk_start;

      // Construct Prompt list for current Batch
      std::vector<PromptMeta> batch_prompts(chunk_prompts.begin() + chunk_start,
                                            chunk_prompts.begin() + chunk_end);

      // Geometry slot size: the most boxes any prompt in THIS batch feeds
      // (0 = text-only batch, decoded exactly as before geometry existed).
      int slot_boxes = 0;
      for (const auto &meta : batch_prompts)
        if (meta.ptr)
          slot_boxes = std::max(
              slot_boxes,
              sam3_geometry_box_count(
                  meta.ptr->boxes.size() +
                      (meta.point_frac > 0.f ? meta.ptr->points.size() : 0),
                  geom_max_boxes_));
      const bool use_geom = slot_boxes > 0;
      const int prompt_len =
          text_ids_shape_[1] + sam3_geometry_slot_len(slot_boxes);

      // a. Gather Vision Features (chunk-local image_idx -> prompt slots)
      gather_vision_features(batch_prompts, current_batch_size, stream);

      // b. Encode Text
      if (!encode_text(batch_prompts, current_batch_size, stream))
        continue;

      // c. Encode Geometry, d. Decode. An engine that accepts the geometry
      // profile but cannot run it (an export that bakes the prompt length)
      // turns geometry off for the model and the batch is decoded text-only,
      // so text segmentation never breaks on account of geometry.
      bool decoded =
          (!use_geom || encode_boxes(batch_prompts, current_batch_size,
                                     slot_boxes, stream)) &&
          decode(current_batch_size, prompt_len, stream);
      if (!decoded && use_geom) {
        reusex::warn("TensorRTSam3: the geometry path failed (prompt_len {}); "
                     "box and point prompts are disabled for this model and "
                     "the batch is decoded text-only",
                     prompt_len);
        geom_max_boxes_ = 0;
        geometry_encoder_trt_.reset();
        decoded = decode(current_batch_size, text_ids_shape_[1], stream);
      }
      if (!decoded)
        continue;

      // e. Postprocess & Collect Results. meta.image_idx is chunk-local (used
      // by postprocess for the per-chunk buffers); results/inputs use the
      // global image index.
      for (int k = 0; k < current_batch_size; ++k) {
        const auto &meta = batch_prompts[k];
        const int global_idx = img_chunk_start + meta.image_idx;

        std::string label = "object";
        if (meta.ptr && !meta.ptr->text.empty())
          label = meta.ptr->text;

        // Per-prompt threshold overrides the frame/global one when set (>= 0).
        const float conf =
            (meta.ptr && meta.ptr->confidence >= 0.0f)
                ? meta.ptr->confidence
                : tensor_inputs[global_idx]->confidence_threshold;
        const int label_idx = sam3_label_value(
            tensor_inputs[global_idx]->label_by_prompt_index, meta.original_idx,
            std::get<2>(text_input_map_[label]));

        // Write result to the corresponding global image index (a point
        // pass collects into its candidates first).
        auto &image_result =
            meta.point_frac > 0.f
                ? point_candidates[{global_idx, meta.original_idx}]
                : results[global_idx];
        const size_t first_new = image_result.size();
        postprocess(image_result, k, meta.image_idx, label, label_idx, conf,
                    return_mask, stream);

        // Box prompts are exemplars: SAM3 returns every similar object.
        // Selection callers keep only the detections the boxes point at.
        if (tensor_inputs[global_idx]->select_box_instances && meta.ptr &&
            (!meta.ptr->boxes.empty() || !meta.ptr->points.empty()) &&
            geom_max_boxes_ > 0) {
          const auto &boxes = meta.ptr->boxes;
          const auto kept = std::remove_if(
              image_result.begin() + first_new, image_result.end(),
              [&](const auto &det) {
                // A point selects a detection only where its mask covers it.
                return !sam3_detection_selected(
                    {det.box.left, det.box.top, det.box.right, det.box.bottom},
                    boxes, meta.ptr->points, [&](float x, float y) {
                      return detection_mask_covers(det, x, y);
                    });
              });
          const auto dropped = std::distance(kept, image_result.end());
          image_result.erase(kept, image_result.end());
          reusex::debug("TensorRTSam3: prompt {} kept {} of {} detection(s) "
                        "selected by its boxes",
                        meta.original_idx, image_result.size() - first_new,
                        image_result.size() - first_new + dropped);
        }
        if (meta.point_frac > 0.f)
          candidate_frac[{global_idx, meta.original_idx}].resize(
              image_result.size(), meta.point_frac);
      }
    }

    // Finalise point prompts: the union of, per click, the most confident
    // detection covering it across all exemplar sizes, and everything the
    // prompt's positive boxes select (sam3_point_prompt_selection).
    for (auto &[key, cands] : point_candidates) {
      const auto &[global_idx, prompt_idx] = key;
      const auto &prompt = tensor_inputs[global_idx]->prompts[prompt_idx];
      std::vector<float> scores;
      std::vector<std::vector<bool>> covers;
      for (const auto &det : cands) {
        scores.push_back(det.score);
        std::vector<bool> row;
        for (const auto &pt : prompt.points)
          row.push_back(det.box.left <= pt[0] && pt[0] <= det.box.right &&
                        det.box.top <= pt[1] && pt[1] <= det.box.bottom &&
                        detection_mask_covers(det, pt[0], pt[1]));
        covers.push_back(std::move(row));
      }
      // Box hits: every pass repeats them, so count the first pass's only.
      const auto &fracs = candidate_frac[key];
      const float first_pass = sam3_point_exemplar_fracs().front();
      // Only positive boxes select; negative ones only drop.
      const bool has_pos_box =
          std::any_of(prompt.boxes.begin(), prompt.boxes.end(),
                      [](const auto &b) { return b.first == "pos"; });
      std::vector<bool> box_selected(cands.size(), false);
      if (has_pos_box)
        for (size_t d = 0; d < cands.size(); ++d)
          box_selected[d] =
              d < fracs.size() && fracs[d] == first_pass &&
              sam3_detection_selected({cands[d].box.left, cands[d].box.top,
                                       cands[d].box.right, cands[d].box.bottom},
                                      prompt.boxes);
      const auto picked =
          sam3_point_prompt_selection(scores, covers, box_selected);
      auto &out = results[global_idx];
      for (const auto d : picked)
        out.push_back(cands[d]);
      reusex::debug("TensorRTSam3: point prompt {} kept {} of {} candidate "
                    "detection(s) across {} exemplar sizes",
                    prompt_idx, picked.size(), cands.size(),
                    sam3_point_exemplar_fracs().size());
    }
  }

  // Make Result Image
  for (size_t i = 0; i < results.size(); i++) {

    auto res_ptr = std::make_unique<TensorRTData>();
    res_ptr->image = cv::Mat(tensor_inputs[i]->image.size(), CV_32S /*CV_16U*/,
                             cv::Scalar(-1));

    reusex::vision::osd::make_labled_image(res_ptr->image, results[i]);
    res_ptr->geometry_prompts_used = geom_max_boxes_ > 0;

    results_img[i] = IDataset::Pair();
    results_img[i].first = std::move(res_ptr);
    results_img[i].second = input[i].second;
  }

  reusex::debug("TensorRTSam3 forward pass completed");
  return results_img;
}

std::string TensorRTSam3::load_bytes_from_file(const std::string &path) {
  // Load bytes from file
  std::ifstream fs(path, std::ios::in | std::ios::binary);
  if (fs.fail()) {
    reusex::error("Failed to open tokenizer file at path: {}", path);
    exit(1);
  }
  reusex::debug("Loading json from file: {}", path);

  std::string data;
  fs.seekg(0, std::ios::end);
  size_t size = static_cast<size_t>(fs.tellg());
  fs.seekg(0, std::ios::beg);
  data.resize(size);
  fs.read(data.data(), size);

  return data;
}

bool TensorRTSam3::load_engines() {
  reusex::info("Loading TensorRT engines");
  AutoDevice device_guard(gpu_id_);
  auto load_engine = [&](const std::string &path,
                         std::shared_ptr<TensorRT::Engine> &engine,
                         const char *name) {
    if (path.empty()) {
      reusex::debug("Skipping {} encoder (empty path)", name);
      return true;
    }
    reusex::debug("Loading {} encoder from: {}", name, path);
    engine = TensorRT::load(path);
    if (!engine) {
      reusex::error("Failed to load {} from path: {}", name, path);
      return false;
    }
    engine->print(path.c_str());
    if (isdynamic_model_)
      isdynamic_model_ = engine->has_dynamic_dim();
    reusex::debug("{} encoder loaded successfully", name);
    return true;
  };

  if (!load_engine(vision_encoder_path_, vision_encoder_trt_, "Vision"))
    return false;
  vision_input_shape_ = vision_encoder_trt_->static_dims(0);
  fpn_feat_0_shape_ = vision_encoder_trt_->static_dims(1);
  input_image_height_ = vision_input_shape_[2];
  input_image_width_ = vision_input_shape_[3];

  if (!load_engine(text_encoder_path_, text_encoder_trt_, "Text"))
    return false;
  text_ids_shape_ = text_encoder_trt_->static_dims(0);

  if (!geometry_encoder_path_.empty()) {
    // Optional: a geometry encoder that fails to load (stale, built for
    // another TensorRT) only costs box/point prompts, never the model.
    if (!load_engine(geometry_encoder_path_, geometry_encoder_trt_,
                     "Geometry") ||
        !geometry_encoder_trt_) {
      reusex::warn("TensorRTSam3: cannot load {}; continuing without box and "
                   "point prompts",
                   geometry_encoder_path_);
      geometry_encoder_trt_.reset();
    } else {
      geom_box_shape_ = geometry_encoder_trt_->static_dims(0);
    }
  }

  if (!load_engine(decoder_path_, decoder_trt_, "Decoder"))
    return false;
  // Geometry prompting needs a geometry encoder that takes a single box and a
  // decoder whose prompt_len profile has room for box + CLS tokens after the
  // text. Engines built from a pre-v2 engine-build.json (decoder fixed at the
  // 32 text tokens, encoder fixed at 8 boxes) fail this and fall back to
  // text-only prompting, which segment_image detects and clips boxes for.
  if (geometry_encoder_trt_) {
    const auto enc_min = geometry_encoder_trt_->profile_dims("input_boxes", 0);
    const auto enc_max = geometry_encoder_trt_->profile_dims("input_boxes", 2);
    const auto dec_max = decoder_trt_->profile_dims("prompt_features", 2);
    const int text_len = text_ids_shape_.size() > 1 ? text_ids_shape_[1] : 32;
    if (enc_min.size() == 3 && enc_max.size() == 3 && dec_max.size() == 3)
      geom_max_boxes_ = sam3_geometry_capacity(
          max_boxes_per_prompt_, enc_min[1], enc_max[1], text_len, dec_max[1]);
    if (geom_max_boxes_ > 0) {
      reusex::info("TensorRTSam3: box prompts enabled (up to {} per prompt)",
                   geom_max_boxes_);
    } else {
      // info, not warn: a text-only run (annotate) never needs geometry;
      // segment_image warns when a request's boxes go unused.
      reusex::info(
          "TensorRTSam3: box prompts disabled — the engines cannot take them "
          "(geometry encoder boxes {}..{}, decoder prompt_len max {}, text "
          "tokens {}). Rebuild the engines from engine-build.json "
          "recipe_version >= 2.",
          enc_min.size() == 3 ? enc_min[1] : -1,
          enc_max.size() == 3 ? enc_max[1] : -1,
          dec_max.size() == 3 ? dec_max[1] : -1, text_len);
      geometry_encoder_trt_.reset();
    }
  }

  auto pred_masks_shape = decoder_trt_->static_dims(6);
  auto pred_boxes_shape = decoder_trt_->static_dims(7);
  num_queries_ = pred_boxes_shape[1];
  mask_width_ = pred_masks_shape[2];
  mask_height_ = pred_masks_shape[3];

  // Initialize fixed GPU memory
  allocate_memory_once();

  // Read the export's mask polarity off the text encoder's own output, so the
  // geometry slot mask matches whatever the decoder was exported with.
  if (geom_max_boxes_ > 0) {
    if (const auto polarity = probe_mask_polarity()) {
      mask_true_is_valid_ = *polarity;
      reusex::debug("TensorRTSam3: token masks use True == {}",
                    mask_true_is_valid_ ? "valid" : "padding");
    } else {
      reusex::warn("TensorRTSam3: cannot tell the export's mask polarity "
                   "from the text encoder; box and point prompts disabled");
      geom_max_boxes_ = 0;
      geometry_encoder_trt_.reset();
    }
  }

  reusex::info("All TensorRT engines loaded successfully");
  return true;
}

void TensorRTSam3::allocate_memory_once() {
  reusex::debug("Allocating GPU memory for TensorRTSam3");
  // 1. Image Batch related (allocated by max_image_batch_)
  affine_matrix_.cpu(max_image_batch_ * 6);
  affine_matrix_.gpu(max_image_batch_ * 6);

  mask_affine_matrix_.cpu(max_image_batch_ * 6);
  mask_affine_matrix_.gpu(max_image_batch_ * 6);

  preprocessed_images_.gpu(max_image_batch_ * 3 * input_image_height_ *
                           input_image_width_);

  // Vision Encoder Outputs
  size_t feat_0_sz_one = static_cast<size_t>(fpn_feat_0_shape_[1]) *
                         fpn_feat_0_shape_[2] * fpn_feat_0_shape_[3];
  size_t feat_0_sz_max_img = max_image_batch_ * feat_0_sz_one;

  fpn_feat_0_.gpu(feat_0_sz_max_img);
  fpn_feat_1_.gpu(feat_0_sz_max_img / 4);
  fpn_feat_2_.gpu(feat_0_sz_max_img / 16);
  fpn_pos_2_.gpu(feat_0_sz_max_img / 16);

  // 2. Decoder Batch related (allocated by max_prompt_batch_)
  size_t feat_0_sz_max_pmt = max_prompt_batch_ * feat_0_sz_one;
  fpn_feat_0_gather_.gpu(feat_0_sz_max_pmt);
  fpn_feat_1_gather_.gpu(feat_0_sz_max_pmt / 4);
  fpn_feat_2_gather_.gpu(feat_0_sz_max_pmt / 16);
  fpn_pos_2_gather_.gpu(feat_0_sz_max_pmt / 16);

  // Text Input
  size_t text_in_sz =
      static_cast<size_t>(max_prompt_batch_) * text_ids_shape_[1];
  text_input_ids_.cpu(text_in_sz);
  text_input_ids_.gpu(text_in_sz);
  text_attention_mask_.cpu(text_in_sz);
  text_attention_mask_.gpu(text_in_sz);

  // Text Feats
  text_features_.gpu(text_in_sz * 256);
  text_mask_.gpu(text_in_sz);

  // Geometry (allocated by max_prompt_batch_ * geom_max_boxes_)
  const bool use_geom = geom_max_boxes_ > 0;
  const size_t geom_slot =
      static_cast<size_t>(sam3_geometry_slot_len(geom_max_boxes_));
  if (use_geom) {
    size_t box_sz =
        static_cast<size_t>(max_prompt_batch_) * geom_max_boxes_ * 4;
    geom_boxes_.cpu(box_sz);
    geom_boxes_.gpu(box_sz);
    geom_labels_.cpu(max_prompt_batch_ * geom_max_boxes_);
    geom_labels_.gpu(max_prompt_batch_ * geom_max_boxes_);

    geom_features_.gpu(max_prompt_batch_ * geom_slot * 256);
    geom_mask_.gpu(max_prompt_batch_ * geom_slot);
    geom_valid_mask_.cpu(max_prompt_batch_ * geom_slot);
    geom_valid_mask_.gpu(max_prompt_batch_ * geom_slot);
  }

  // Decoder Input (Prompt feats): text tokens + the largest geometry slot.
  size_t total_prompt_len = text_ids_shape_[1] + geom_slot;
  prompt_features_.gpu(max_prompt_batch_ * total_prompt_len * 256);
  prompt_mask_.gpu(max_prompt_batch_ * total_prompt_len);

  // Decoder Output
  pred_masks_.gpu(max_prompt_batch_ * num_queries_ * mask_height_ *
                  mask_width_);
  pred_boxes_.gpu(max_prompt_batch_ * num_queries_ * 4);
  pred_logits_.gpu(max_prompt_batch_ * num_queries_);
  presence_logits_.gpu(max_prompt_batch_ * 1);

  // Postprocess Buffers
  size_t post_sz = static_cast<size_t>(max_prompt_batch_) * num_queries_;
  filter_boxes_.cpu(post_sz * 4);
  filter_boxes_.gpu(post_sz * 4);
  filter_scores_.cpu(post_sz);
  filter_scores_.gpu(post_sz);
  filter_indices_.cpu(post_sz);
  filter_indices_.gpu(post_sz);
  box_count_.cpu(1);
  box_count_.gpu(1);

  // Mask Postprocess (assume each prompt produces at most one valid mask to
  // estimate buffer) Or directly allocate enough GPU memory. For safety,
  // allocate by max_prompt_batch_ * num_queries_ Allocation: usually only a
  // few valid boxes. Allocate a safe value, such as 256MB, or check
  // dynamically. Here allocate the maximum possible. Assume each prompt
  // produces 1 mask, maximum max_prompt_batch_ masks. If many masks,
  // process in a loop. Here, box_affine_matrices_ is for Postprocess kernel
  // use
  box_affine_matrices_.cpu(post_sz * 6);
  box_affine_matrices_.gpu(post_sz * 6);

  // // Mask Buffer: pre-allocate a large pool, e.g., 512MB
  // size_t mask_pool_size = 256 * 1024 * 1024;
  // mask_buffer_.gpu(mask_pool_size);
  // mask_buffer_.cpu(mask_pool_size);
  reusex::debug("GPU memory allocation completed");
}

// Only call geometry model to store results in geom_features_cache_ and
// geom_mask__cache_
bool TensorRTSam3::setup_geometry_input(
    const cv::Mat &image, const std::string &label,
    const std::vector<std::pair<std::string, std::array<float, 4>>> &boxes) {
  if (geom_max_boxes_ <= 0) {
    return false;
  }

  AutoDevice device_guard(gpu_id_);

  // step 1 : Image preprocessing
  int ibatch = 0;
  TensorRTData input{};
  input.image = image;
  // Sam3Input input = Sam3Input(image);
  preprocess(input, 0, nullptr);
  // step2 : encode image
  if (!encode_image(1, nullptr)) {
    return false;
  }

  Sam3PromptUnit prompt_unit = Sam3PromptUnit(label, boxes);
  PromptMeta meta = {0, 0, &prompt_unit};
  std::vector<PromptMeta> batch_meta;
  batch_meta.push_back(meta);

  gather_vision_features(batch_meta, 1, nullptr);

  // step3 : encode_boxes
  if (!encode_boxes(batch_meta, 1,
                    sam3_geometry_box_count(boxes.size(), geom_max_boxes_),
                    nullptr)) {
    return false;
  }
  geom_features_cache_[label] = std::make_shared<tensor::Memory<float>>();
  geom_features_cache_[label]->gpu(geom_features_.gpu_bytes());
  geom_mask_cache_[label] = std::make_shared<tensor::Memory<bool>>();
  geom_mask_cache_[label]->gpu(geom_mask_.gpu_bytes());
  cudaStream_t s = (cudaStream_t) nullptr;
  cudaMemcpyAsync(geom_features_cache_[label]->gpu(), geom_features_.gpu(),
                  geom_features_.gpu_bytes(), cudaMemcpyDeviceToDevice, s);
  cudaMemcpyAsync(geom_mask_cache_[label]->gpu(), geom_mask_.gpu(),
                  geom_mask_.gpu_bytes(), cudaMemcpyDeviceToDevice, s);
  cudaStreamSynchronize(s);
  return true;
}

std::optional<bool> TensorRTSam3::probe_mask_polarity() {
  // "a" tokenizes to [start, a, end]: 3 real tokens, 29 padding.
  const auto [ids, mask] = make_ids(tokenizer_->Encode("a"));
  const int seq_len = text_ids_shape_[1];
  int real = 0;
  for (int i = 0; i < seq_len; ++i)
    real += mask[i] != 0;
  std::copy_n(ids.data(), seq_len, text_input_ids_.cpu());
  std::copy_n(mask.data(), seq_len, text_attention_mask_.cpu());

  cudaStream_t s = nullptr;
  cudaMemcpyAsync(text_input_ids_.gpu(), text_input_ids_.cpu(),
                  seq_len * sizeof(int64_t), cudaMemcpyHostToDevice, s);
  cudaMemcpyAsync(text_attention_mask_.gpu(), text_attention_mask_.cpu(),
                  seq_len * sizeof(int64_t), cudaMemcpyHostToDevice, s);
  set_binding_dim(text_encoder_trt_, 0, {1, seq_len});
  set_binding_dim(text_encoder_trt_, 1, {1, seq_len});
  if (!text_encoder_trt_->forward(
          {{"input_ids", text_input_ids_.gpu()},
           {"attention_mask", text_attention_mask_.gpu()},
           {"text_features", text_features_.gpu()},
           {"text_mask", text_mask_.gpu()}},
          s))
    return std::nullopt;
  std::unique_ptr<bool[]> host(new bool[seq_len]);
  cudaMemcpyAsync(host.get(), text_mask_.gpu(), seq_len * sizeof(bool),
                  cudaMemcpyDeviceToHost, s);
  if (cudaStreamSynchronize(s) != cudaSuccess)
    return std::nullopt;
  std::vector<std::uint8_t> out(host.get(), host.get() + seq_len);
  return sam3_mask_true_is_valid(out, real);
}

void TensorRTSam3::set_binding_dim(std::shared_ptr<TensorRT::Engine> &engine,
                                   int idx, const std::vector<int> &dims) {
  // Gate on the ENGINE's own dynamic flag, not the model-wide AND
  // (isdynamic_model_). A statically-shaped engine (e.g. a vision encoder
  // built with a fixed batch) must not suppress run-dim setting on the other,
  // dynamically-shaped engines (text/decoder), which would otherwise fail with
  // "input shapes not specified".
  if (engine && engine->has_dynamic_dim())
    engine->set_run_dims(idx, dims);
}

void TensorRTSam3::preprocess(const TensorRTData &input, int ibatch,
                              void *stream) {
  reusex::trace("Preprocessing image for batch index {}", ibatch);
  cudaStream_t s = (cudaStream_t)stream;
  // const cv::Mat &img = input.image;
  const cv::Mat &img = input.image;
  common_tensor::Image img_tensor = common_tensor::cvimg(img);

  // Record original size
  original_image_sizes_[ibatch] = {img_tensor.width, img_tensor.height};

  affine::ResizeMatrix matrix;
  matrix.compute(std::make_tuple(img_tensor.width, img_tensor.height),
                 std::make_tuple(input_image_width_, input_image_height_));

  size_t size_image = img_tensor.width * img_tensor.height * 3;
  uint8_t *h_buf = original_images_buf_[ibatch]->cpu(size_image);

  if (img.isContinuous()) {
    memcpy(h_buf, img.data, size_image);
  } else {
    int w_bytes = img_tensor.width * 3;
    for (int h = 0; h < img_tensor.height; ++h)
      memcpy(h_buf + h * w_bytes, img.ptr<uint8_t>(h), w_bytes);
  }

  float *h_mat = affine_matrix_.cpu() + ibatch * 6;
  memcpy(h_mat, matrix.d2i, sizeof(matrix.d2i));

  cudaMemcpyAsync(original_images_buf_[ibatch]->gpu(size_image), h_buf,
                  size_image, cudaMemcpyHostToDevice, s);
  cudaMemcpyAsync(affine_matrix_.gpu() + ibatch * 6, h_mat, sizeof(matrix.d2i),
                  cudaMemcpyHostToDevice, s);

  // Mask Affine Matrix
  affine::ResizeMatrix mask_m;
  mask_m.compute(std::make_tuple(mask_width_, mask_height_),
                 std::make_tuple(img_tensor.width, img_tensor.height));
  memcpy(mask_affine_matrix_.cpu() + ibatch * 6, mask_m.d2i,
         sizeof(mask_m.d2i));
  cudaMemcpyAsync(mask_affine_matrix_.gpu() + ibatch * 6, mask_m.d2i,
                  sizeof(mask_m.d2i), cudaMemcpyHostToDevice, s);

  warp_affine_bilinear_and_normalize_plane(
      original_images_buf_[ibatch]->gpu(), img_tensor.width * 3,
      img_tensor.width, img_tensor.height,
      preprocessed_images_.gpu() +
          ibatch * 3 * input_image_height_ * input_image_width_,
      input_image_width_, input_image_height_,
      affine_matrix_.gpu() + ibatch * 6, 114, preprocess_norm_, s);
}

bool TensorRTSam3::encode_image(int batch_size, void *stream) {
  reusex::debug("Encoding {} images with vision encoder", batch_size);
  // Set input dimensions
  set_binding_dim(vision_encoder_trt_, 0,
                  {batch_size, 3, input_image_height_, input_image_width_});

  bool success =
      vision_encoder_trt_->forward({{"images", preprocessed_images_.gpu()},
                                    {"fpn_feat_0", fpn_feat_0_.gpu()},
                                    {"fpn_feat_1", fpn_feat_1_.gpu()},
                                    {"fpn_feat_2", fpn_feat_2_.gpu()},
                                    {"fpn_pos_2", fpn_pos_2_.gpu()}},
                                   (cudaStream_t)stream);
  if (!success) {
    reusex::error("Vision encoder forward pass failed");
  }
  return success;
}

// Core optimization: Gather mode
void TensorRTSam3::gather_vision_features(
    const std::vector<PromptMeta> &batch_prompts, int batch_size,
    void *stream) {
  cudaStream_t s = (cudaStream_t)stream;

  size_t sz_0 = static_cast<size_t>(fpn_feat_0_shape_[1]) *
                fpn_feat_0_shape_[2] * fpn_feat_0_shape_[3];
  size_t sz_1 = sz_0 / 4;
  size_t sz_2 = sz_0 / 16;

  // Iterate through each Prompt in the current Batch
  for (int i = 0; i < batch_size; ++i) {
    int img_idx = batch_prompts[i].image_idx;

    // Source address: offset in Image queue
    float *src_0 = fpn_feat_0_.gpu() + img_idx * sz_0;
    float *src_1 = fpn_feat_1_.gpu() + img_idx * sz_1;
    float *src_2 = fpn_feat_2_.gpu() + img_idx * sz_2;
    float *src_p = fpn_pos_2_.gpu() + img_idx * sz_2;

    // Target address: offset in Prompt queue (i)
    float *dst_0 = fpn_feat_0_gather_.gpu() + i * sz_0;
    float *dst_1 = fpn_feat_1_gather_.gpu() + i * sz_1;
    float *dst_2 = fpn_feat_2_gather_.gpu() + i * sz_2;
    float *dst_p = fpn_pos_2_gather_.gpu() + i * sz_2;

    // Asynchronous copy
    // The number of copies here equals batch_size, usually < 100, overhead
    // is controllable. For ultimate optimization, a Kernel can be written,
    // but it's simpler to maintain in C++ logic
    cudaMemcpyAsync(dst_0, src_0, sz_0 * sizeof(float),
                    cudaMemcpyDeviceToDevice, s);
    cudaMemcpyAsync(dst_1, src_1, sz_1 * sizeof(float),
                    cudaMemcpyDeviceToDevice, s);
    cudaMemcpyAsync(dst_2, src_2, sz_2 * sizeof(float),
                    cudaMemcpyDeviceToDevice, s);
    cudaMemcpyAsync(dst_p, src_p, sz_2 * sizeof(float),
                    cudaMemcpyDeviceToDevice, s);
  }
}

bool TensorRTSam3::encode_text(const std::vector<PromptMeta> &batch_prompts,
                               int batch_size, void *stream) {
  reusex::trace("Encoding text for batch_size={}", batch_size);
  int seq_len = 32;
  int64_t *h_ids = text_input_ids_.cpu();
  int64_t *h_mask = text_attention_mask_.cpu();

  std::array<int64_t, 32> def_ids;
  def_ids.fill(49407);
  std::array<int64_t, 32> def_mask = {0};
  def_mask[0] = 1;

  for (int i = 0; i < batch_size; ++i) {
    const Sam3PromptUnit *prompt = batch_prompts[i].ptr;
    const int64_t *src_ids = def_ids.data();
    const int64_t *src_mask = def_mask.data();

    if (prompt && text_input_map_.count(prompt->text)) {
      src_ids = std::get<0>(text_input_map_[prompt->text]).data();
      src_mask = std::get<1>(text_input_map_[prompt->text]).data();
    }

    memcpy(h_ids + i * seq_len, src_ids, seq_len * sizeof(int64_t));
    memcpy(h_mask + i * seq_len, src_mask, seq_len * sizeof(int64_t));
  }

  cudaStream_t s = (cudaStream_t)stream;
  cudaMemcpyAsync(text_input_ids_.gpu(), h_ids,
                  batch_size * seq_len * sizeof(int64_t),
                  cudaMemcpyHostToDevice, s);
  cudaMemcpyAsync(text_attention_mask_.gpu(), h_mask,
                  batch_size * seq_len * sizeof(int64_t),
                  cudaMemcpyHostToDevice, s);

  // Set dimensions
  set_binding_dim(text_encoder_trt_, 0, {batch_size, seq_len});
  set_binding_dim(text_encoder_trt_, 1, {batch_size, seq_len});

  return text_encoder_trt_->forward(
      {{"input_ids", text_input_ids_.gpu()},
       {"attention_mask", text_attention_mask_.gpu()},
       {"text_features", text_features_.gpu()},
       {"text_mask", text_mask_.gpu()}},
      s);
}

bool TensorRTSam3::encode_boxes(const std::vector<PromptMeta> &batch_prompts,
                                int batch_size, int slot_boxes, void *stream) {
  if (!geometry_encoder_trt_ || slot_boxes <= 0)
    return true;
  cudaStream_t s = (cudaStream_t)stream;

  constexpr int feat_dim = 256;
  const int slot_len = sam3_geometry_slot_len(slot_boxes);
  const size_t sz_2 = static_cast<size_t>(fpn_feat_0_shape_[1]) *
                      fpn_feat_0_shape_[2] * fpn_feat_0_shape_[3] / 16;
  const int feat_h = fpn_feat_0_shape_[2] / 4;
  const int feat_w = fpn_feat_0_shape_[3] / 4;

  // Unused slot tokens are masked, but they still enter the attention
  // matmuls: zero them so stale memory can never inject a NaN.
  cudaMemsetAsync(
      geom_features_.gpu(), 0,
      static_cast<size_t>(batch_size) * slot_len * feat_dim * sizeof(float), s);

  float *h_boxes = geom_boxes_.cpu();
  int64_t *h_labels = geom_labels_.cpu();
  bool *h_valid = geom_valid_mask_.cpu();

  for (int i = 0; i < batch_size; ++i) {
    const Sam3PromptUnit *prompt = batch_prompts[i].ptr;
    const int img_idx = batch_prompts[i].image_idx;
    const int iw = original_image_sizes_[img_idx].first;
    const int ih = original_image_sizes_[img_idx].second;
    // The geometry encoder takes boxes only: drawn boxes first, then each
    // click point as a positive exemplar box around it.
    std::vector<BoxPrompt> geometry;
    if (prompt) {
      geometry = prompt->boxes;
      if (batch_prompts[i].point_frac > 0.f)
        for (const auto &pt : prompt->points)
          geometry.emplace_back(
              "pos",
              sam3_point_exemplar_box(pt, iw, ih, batch_prompts[i].point_frac));
    }
    const int n = sam3_geometry_box_count(geometry.size(), slot_boxes);

    const auto mask = sam3_geometry_slot_mask(n, slot_len);
    for (int t = 0; t < slot_len; ++t)
      h_valid[static_cast<size_t>(i) * slot_len + t] =
          sam3_mask_value(mask[t] != 0, mask_true_is_valid_);
    if (n == 0)
      continue;

    // Each prompt owns its own host/device region, so the async copies of
    // one prompt never race the host writes of the next.
    float *hb = h_boxes + static_cast<size_t>(i) * geom_max_boxes_ * 4;
    int64_t *hl = h_labels + static_cast<size_t>(i) * geom_max_boxes_;
    float *db =
        geom_boxes_.gpu() + static_cast<size_t>(i) * geom_max_boxes_ * 4;
    int64_t *dl = geom_labels_.gpu() + static_cast<size_t>(i) * geom_max_boxes_;
    for (int k = 0; k < n; ++k) {
      const auto &[polarity, xyxy] = geometry[k];
      const auto b = sam3_box_cxcywh(xyxy, iw, ih);
      std::copy(b.begin(), b.end(), hb + k * 4);
      hl[k] = sam3_box_label(polarity);
    }
    cudaMemcpyAsync(db, hb, n * 4 * sizeof(float), cudaMemcpyHostToDevice, s);
    cudaMemcpyAsync(dl, hl, n * sizeof(int64_t), cudaMemcpyHostToDevice, s);

    if (!geometry_encoder_trt_->set_run_dims(0, {1, n, 4}) ||
        !geometry_encoder_trt_->set_run_dims(1, {1, n}) ||
        !geometry_encoder_trt_->set_run_dims(
            2, {1, fpn_feat_0_shape_[1], feat_h, feat_w}) ||
        !geometry_encoder_trt_->set_run_dims(
            3, {1, fpn_feat_0_shape_[1], feat_h, feat_w})) {
      reusex::error("TensorRTSam3: geometry encoder rejected {} box(es)", n);
      return false;
    }

    // Output [1, n + 1, 256] lands at the head of this prompt's slot; the
    // engine's geometry_mask output is scratch (see geom_valid_mask_).
    const bool ok = geometry_encoder_trt_->forward(
        {{"input_boxes", db},
         {"input_boxes_labels", dl},
         {"fpn_feat_2", fpn_feat_2_gather_.gpu() + i * sz_2},
         {"fpn_pos_2", fpn_pos_2_gather_.gpu() + i * sz_2},
         {"geometry_features",
          geom_features_.gpu() + static_cast<size_t>(i) * slot_len * feat_dim},
         {"geometry_mask",
          geom_mask_.gpu() + static_cast<size_t>(i) * slot_len}},
        s);
    if (!ok) {
      reusex::error("TensorRTSam3: geometry encoder forward failed");
      return false;
    }
  }

  cudaMemcpyAsync(geom_valid_mask_.gpu(), h_valid,
                  static_cast<size_t>(batch_size) * slot_len * sizeof(bool),
                  cudaMemcpyHostToDevice, s);
  return true;
}

bool TensorRTSam3::decode(int batch_size, int prompt_len, void *stream) {
  reusex::debug("Decoding with batch_size={}, prompt_len={}", batch_size,
                prompt_len);
  int text_len = text_ids_shape_[1];
  int feat_dim = 256;
  size_t feat_sz = feat_dim * sizeof(float);
  size_t mask_sz = sizeof(bool);

  char *d_prompt = (char *)prompt_features_.gpu();
  char *d_prompt_m = (char *)prompt_mask_.gpu();
  char *d_text = (char *)text_features_.gpu();
  char *d_text_m = (char *)text_mask_.gpu();
  char *d_geom = (char *)geom_features_.gpu();
  char *d_geom_m = (char *)geom_valid_mask_.gpu();

  cudaStream_t s = (cudaStream_t)stream;

  // Concatenate Prompt Features
  for (int i = 0; i < batch_size; ++i) {
    size_t prompt_off = static_cast<size_t>(i) * prompt_len * feat_sz;
    size_t prompt_m_off = static_cast<size_t>(i) * prompt_len * mask_sz;

    cudaMemcpyAsync(d_prompt + prompt_off, d_text + i * text_len * feat_sz,
                    text_len * feat_sz, cudaMemcpyDeviceToDevice, s);
    cudaMemcpyAsync(d_prompt_m + prompt_m_off,
                    d_text_m + i * text_len * mask_sz, text_len * mask_sz,
                    cudaMemcpyDeviceToDevice, s);

    if (prompt_len > text_len) {
      size_t geom_len = prompt_len - text_len;
      cudaMemcpyAsync(d_prompt + prompt_off + text_len * feat_sz,
                      d_geom + i * geom_len * feat_sz, geom_len * feat_sz,
                      cudaMemcpyDeviceToDevice, s);
      cudaMemcpyAsync(d_prompt_m + prompt_m_off + text_len * mask_sz,
                      d_geom_m + i * geom_len * mask_sz, geom_len * mask_sz,
                      cudaMemcpyDeviceToDevice, s);
    }
  }

  set_binding_dim(decoder_trt_, 0,
                  {batch_size, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2],
                   fpn_feat_0_shape_[3]});
  set_binding_dim(decoder_trt_, 1,
                  {batch_size, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 2,
                   fpn_feat_0_shape_[3] / 2});
  set_binding_dim(decoder_trt_, 2,
                  {batch_size, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 4,
                   fpn_feat_0_shape_[3] / 4});
  set_binding_dim(decoder_trt_, 3,
                  {batch_size, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 4,
                   fpn_feat_0_shape_[3] / 4});
  set_binding_dim(decoder_trt_, 4, {batch_size, prompt_len, 256});
  set_binding_dim(decoder_trt_, 5, {batch_size, prompt_len});

  // Use features after Gather
  return decoder_trt_->forward({{"fpn_feat_0", fpn_feat_0_gather_.gpu()},
                                {"fpn_feat_1", fpn_feat_1_gather_.gpu()},
                                {"fpn_feat_2", fpn_feat_2_gather_.gpu()},
                                {"fpn_pos_2", fpn_pos_2_gather_.gpu()},
                                {"prompt_features", prompt_features_.gpu()},
                                {"prompt_mask", prompt_mask_.gpu()},
                                {"pred_masks", pred_masks_.gpu()},
                                {"pred_boxes", pred_boxes_.gpu()},
                                {"pred_logits", pred_logits_.gpu()},
                                {"presence_logits", presence_logits_.gpu()}},
                               s);
}

void TensorRTSam3::postprocess(InferResult &image_result, int batch_idx,
                               int image_idx, const std::string &label,
                               const int label_id, float confidence_threshold,
                               bool return_mask, void *stream) {
  reusex::trace("Postprocessing results for image index {}, batch index {}",
                image_idx, batch_idx);
  cudaStream_t s = (cudaStream_t)stream;

  // Pointer offset (based on index in current Batch: batch_idx)
  float *d_pred_masks =
      pred_masks_.gpu() + batch_idx * num_queries_ * mask_height_ * mask_width_;
  float *d_pred_boxes = pred_boxes_.gpu() + batch_idx * num_queries_ * 4;
  float *d_pred_logits = pred_logits_.gpu() + batch_idx * num_queries_;
  float *d_presence = presence_logits_.gpu() + batch_idx;

  float *d_filter_boxes = filter_boxes_.gpu() + batch_idx * num_queries_ * 4;
  float *d_filter_scores = filter_scores_.gpu() + batch_idx * num_queries_;
  int *d_filter_indices = filter_indices_.gpu() + batch_idx * num_queries_;

  cudaMemsetAsync(box_count_.gpu(), 0, sizeof(int), s);

  // Filter
  sam3_postprocess_plane(
      d_pred_masks, d_pred_boxes, d_pred_logits, d_presence, d_filter_boxes,
      d_filter_indices, d_filter_scores, box_count_.gpu(), num_queries_,
      mask_height_, mask_width_, original_image_sizes_[image_idx].first,
      original_image_sizes_[image_idx].second, confidence_threshold, s);

  cudaMemcpyAsync(box_count_.cpu(), box_count_.gpu(), sizeof(int),
                  cudaMemcpyDeviceToHost, s);
  cudaStreamSynchronize(s);
  int count = *box_count_.cpu();

  if (count > 0) {
    std::vector<float> h_boxes(count * 4);
    std::vector<float> h_scores(count);
    std::vector<int> h_indices(count);

    cudaMemcpyAsync(h_boxes.data(), d_filter_boxes, count * 4 * sizeof(float),
                    cudaMemcpyDeviceToHost, s);
    cudaMemcpyAsync(h_scores.data(), d_filter_scores, count * sizeof(float),
                    cudaMemcpyDeviceToHost, s);
    cudaMemcpyAsync(h_indices.data(), d_filter_indices, count * sizeof(int),
                    cudaMemcpyDeviceToHost, s);

    if (!return_mask) {
      for (int i = 0; i < count; ++i) {
        float *b = h_boxes.data() + i * 4;
        reusex::trace("Creating Box for Image {}, Box {}: [{}, {}, {}, {}], "
                      "Score: {}, Label: '{}' ({})",
                      image_idx, i, b[0], b[1], b[2], b[3], h_scores[i], label,
                      label_id);
        image_result.push_back(object::create_box(
            b[0], b[1], b[2], b[3], h_scores[i], label_id, label));
      }
      return;
    }

    float *h_base_matrix = mask_affine_matrix_.cpu() + image_idx * 6;
    float *h_box_matrices = box_affine_matrices_.cpu();

    size_t total_mask_pixels = 0;
    std::vector<size_t> mask_offsets(count);
    std::vector<cv::Size> mask_sizes(count);

    for (int i = 0; i < count; ++i) {
      float *b = h_boxes.data() + i * 4;
      int x1 = std::max(0, (int)b[0]);
      int y1 = std::max(0, (int)b[1]);
      int x2 = std::min(original_image_sizes_[image_idx].first, (int)b[2]);
      int y2 = std::min(original_image_sizes_[image_idx].second, (int)b[3]);

      int box_w = std::max(1, x2 - x1);
      int box_h = std::max(1, y2 - y1);

      mask_sizes[i] = cv::Size(box_w, box_h);
      mask_offsets[i] = total_mask_pixels;
      total_mask_pixels += box_w * box_h;

      float *m_dst = h_box_matrices + i * 6;
      m_dst[0] = h_base_matrix[0];
      m_dst[1] = h_base_matrix[1];
      m_dst[3] = h_base_matrix[3];
      m_dst[4] = h_base_matrix[4];
      m_dst[2] =
          h_base_matrix[0] * x1 + h_base_matrix[1] * y1 + h_base_matrix[2];
      m_dst[5] =
          h_base_matrix[3] * x1 + h_base_matrix[4] * y1 + h_base_matrix[5];
    }

    mask_buffer_.gpu(total_mask_pixels);
    mask_buffer_.cpu(total_mask_pixels);

    cudaMemcpyAsync(box_affine_matrices_.gpu(), box_affine_matrices_.cpu(),
                    count * 6 * sizeof(float), cudaMemcpyHostToDevice, s);

    for (int i = 0; i < count; ++i) {
      int idx = h_indices[i];
      float *src = d_pred_masks + idx * mask_height_ * mask_width_;
      uint8_t *dst = mask_buffer_.gpu() + mask_offsets[i];
      float *d_matrix = box_affine_matrices_.gpu() + i * 6;

      warp_affine_bilinear_single_channel_mask_plane(
          src, mask_width_, mask_width_, mask_height_, dst, mask_sizes[i].width,
          mask_sizes[i].height, d_matrix, 0, s);
    }

    cudaMemcpyAsync(mask_buffer_.cpu(), mask_buffer_.gpu(), total_mask_pixels,
                    cudaMemcpyDeviceToHost, s);
    cudaStreamSynchronize(s);

    for (int i = 0; i < count; ++i) {
      float *b = h_boxes.data() + i * 4;
      uint8_t *mask_ptr = mask_buffer_.cpu() + mask_offsets[i];
      cv::Mat bin_mask(mask_sizes[i].height, mask_sizes[i].width, CV_8U,
                       mask_ptr);

      reusex::trace(
          "Creating SegmentationBox for Image {}, Box {}: [{}, {}, {}, {}], "
          "Score: {}, Label: '{}' ({})",
          image_idx, i, b[0], b[1], b[2], b[3], h_scores[i], label, label_id);
      image_result.push_back(object::create_segmentation_box(
          b[0], b[1], b[2], b[3], bin_mask.clone(), h_scores[i], label_id,
          label));
    }
  }
}

/*
InferResultArray TensorRTSam3::forwards(const std::vector<Sam3Input>
&inputs, const std::string &geom_label, bool return_mask, void *stream) { if
(inputs.empty()) return {};

  // Check if cache exists
  if (geom_mask_cache_.count(geom_label) == 0 ||
      geom_features_cache_.count(geom_label) == 0) {
    reusex::error("Geometry cache not found for label: {}", geom_label);
    return {};
  }

  if (inputs.size() > (size_t)max_image_batch_) {
    reusex::error("Input image batch size ({}) exceeds maximum supported
({}). " "Returning empty.", inputs.size(), max_image_batch_); return
InferResultArray(inputs.size());
  }

  AutoDevice device_guard(gpu_id_);
  cudaStream_t s = (cudaStream_t)stream;

  // 1. Vision Encoder preprocessing and inference
  int num_images = inputs.size();
  for (int i = 0; i < num_images; ++i)
    preprocess(inputs[i], i, stream);

  if (!encode_image(num_images, stream)) {
    return InferResultArray(num_images);
  }

  InferResultArray results(num_images);

  int geom_seq_len = (max_boxes_per_prompt_ + 1);
  size_t single_geom_feat_bytes = geom_seq_len * 256 * sizeof(float);
  size_t single_geom_mask_bytes = geom_seq_len * sizeof(bool);

  // Total Prompt length = Text (32) + Geom
  int total_prompt_len = text_ids_shape_[1] + geom_seq_len;

  // Get pointer to cached data (assume cache stores result for batch=1 at
start
  // of GPU memory)
  Sam3PromptUnit text_unit(geom_label, {});

  float *cached_feat_mem = geom_features_cache_[geom_label]->gpu();
  bool *cached_mask_mem = geom_mask_cache_[geom_label]->gpu();

  for (int chunk_start = 0; chunk_start < num_images;
       chunk_start += max_prompt_batch_) {
    int chunk_end = std::min(chunk_start + max_prompt_batch_, num_images);
    int current_batch_size = chunk_end - chunk_start;

    std::vector<PromptMeta> batch_meta;
    batch_meta.reserve(current_batch_size);
    for (int i = 0; i < current_batch_size; ++i) {
      batch_meta.push_back({chunk_start + i, -1, &text_unit});
    }

    gather_vision_features(batch_meta, current_batch_size, stream);

    if (!encode_text(batch_meta, current_batch_size, stream))
      continue;

    // Overwrite Geometry features (read from cache)
    // This step remains unchanged, still uses previously cached Box
features for (int i = 0; i < current_batch_size; ++i) { float *dst_feat =
geom_features_.gpu() + i * geom_seq_len * 256; bool *dst_mask =
geom_mask_.gpu() + i * geom_seq_len;

      cudaMemcpyAsync(dst_feat, cached_feat_mem, single_geom_feat_bytes,
                      cudaMemcpyDeviceToDevice, s);
      cudaMemcpyAsync(dst_mask, cached_mask_mem, single_geom_mask_bytes,
                      cudaMemcpyDeviceToDevice, s);
    }

    if (!decode(current_batch_size, total_prompt_len, stream))
      continue;

    for (int k = 0; k < current_batch_size; ++k) {
      int image_global_idx = chunk_start + k;
      float conf = inputs[image_global_idx].confidence_threshold;

      // Store result back to corresponding input index
      postprocess(results[image_global_idx], k, image_global_idx,
geom_label, conf, return_mask, stream);
    }
  }
  return results;
}


InferResultArray TensorRTSam3::forwards(const std::vector<Sam3Input>
&inputs, bool return_mask, void *stream) { if (inputs.empty()) return {};

// 1. Check if number of images exceeds limit
if (inputs.size() > (size_t)max_image_batch_) {
  reusex::error("Input image batch size ({}) exceeds maximum supported
({}). " "Returning empty.", inputs.size(), max_image_batch_); return
InferResultArray(inputs.size()); // Return empty result
}

AutoDevice device_guard(gpu_id_);

std::vector<PromptMeta> all_prompts;
int max_boxes_input = 0;

for (size_t i = 0; i < inputs.size(); ++i) {
  if (inputs[i].prompts.empty()) {
    all_prompts.push_back({(int)i, -1, nullptr});
  } else {
    for (size_t j = 0; j < inputs[i].prompts.size(); ++j) {
      all_prompts.push_back({(int)i, (int)j, &inputs[i].prompts[j]});
      if ((int)inputs[i].prompts[j].boxes.size() > max_boxes_input) {
        max_boxes_input = (int)inputs[i].prompts[j].boxes.size();
      }
    }
  }
}

// 3. Vision Encoder (process all images at once)
int num_images = inputs.size();
for (int i = 0; i < num_images; ++i)
  preprocess(inputs[i], i, stream);

if (!encode_image(num_images, stream)) {
  return InferResultArray(num_images);
}

// 4. Decoder batch loop (Batch Splitting)
InferResultArray results(num_images);
int total_prompts = all_prompts.size();
bool use_geom = !geometry_encoder_path_.empty() && max_boxes_input > 0;

// If actual Box count exceeds preset GPU memory allocation, truncate
(prevent
// overflow)
if (max_boxes_input > max_boxes_per_prompt_)
  max_boxes_input = max_boxes_per_prompt_;

int prompt_len = text_ids_shape_[1] + (use_geom ? (max_boxes_input + 1) :
0);

for (int chunk_start = 0; chunk_start < total_prompts;
     chunk_start += max_prompt_batch_) {
  int chunk_end = std::min(chunk_start + max_prompt_batch_, total_prompts);
  int current_batch_size = chunk_end - chunk_start;

  // Construct Prompt list for current Batch
  std::vector<PromptMeta> batch_prompts(all_prompts.begin() + chunk_start,
                                        all_prompts.begin() + chunk_end);

  // a. Gather Vision Features (gather from N images' features to M Prompts
in
  // current batch Prompt features)
  gather_vision_features(batch_prompts, current_batch_size, stream);

  // b. Encode Text
  if (!encode_text(batch_prompts, current_batch_size, stream))
    continue;

  // c. Encode Geometry
  if (use_geom) {
    if (!encode_boxes(batch_prompts, current_batch_size, max_boxes_input,
                      stream))
      continue;
  }

  // d. Decode
  if (!decode(current_batch_size, prompt_len, stream))
    continue;

  // e. Postprocess & Collect Results
  for (int k = 0; k < current_batch_size; ++k) {
    const auto &meta = batch_prompts[k];
    std::string label = "object";
    if (meta.ptr && !meta.ptr->text.empty())
      label = meta.ptr->text;

    float conf = inputs[meta.image_idx].confidence_threshold;

    // Write result to corresponding image_idx
    postprocess(results[meta.image_idx], k, meta.image_idx, label, conf,
                return_mask, stream);
  }
}

return results;
}
*/

} // namespace reusex::vision::tensor_rt

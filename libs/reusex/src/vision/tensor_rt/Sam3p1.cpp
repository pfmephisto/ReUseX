// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/tensor_rt/Sam3p1.hpp"
#include "core/logging.hpp"
#include "vision/common/create_object.hpp"
#include "vision/common/image.hpp"
#include "vision/osd/osd.hpp"
#include "vision/tensor_rt/Data.hpp"
#include "vision/tensor_rt/common/affine.hpp"
#include "vision/tensor_rt/common/device.hpp"
#include "vision/tensor_rt/kernels/postprocess.cuh"
#include "vision/tensor_rt/kernels/process_kernel_warp.hpp"
#include <cstdlib>

#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <range/v3/view/take.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <span>

// make_ids is declared/defined in Sam3.cpp (same translation-unit style layout,
// but a separate TU). Re-declare the free function here so we can reuse the
// exact SAM tokenizer packing without duplicating it — the linker resolves it
// against Sam3.cpp's definition.
using ArrayInt64 = std::array<int64_t, 32>;
std::pair<ArrayInt64, ArrayInt64> make_ids(const std::vector<int32_t> &ids);

namespace reusex::vision::tensor_rt {

namespace object = ::reusex::vision::common::object;
namespace common_tensor = ::reusex::vision::common::tensor;

std::unique_ptr<TensorRTSam3p1>
TensorRTSam3p1::create(const std::filesystem::path &model_path) {
  reusex::info("Creating TensorRTSam3p1 from model path: {}", model_path);

  std::filesystem::path vision_encoder_path =
      model_path / "vision-encoder.engine";
  std::filesystem::path text_encoder_path = model_path / "text-encoder.engine";
  std::filesystem::path decoder_path = model_path / "decoder.engine";
  std::filesystem::path geometry_encoder_path =
      model_path / "geometry-encoder.engine";
  std::filesystem::path memory_encoder_path =
      model_path / "tracker-memory-encoder.engine";
  std::filesystem::path memory_attention_path =
      model_path / "tracker-memory-attention.engine";
  std::filesystem::path tokenizer_path = model_path / "tokenizer.json";
  std::filesystem::path meta_path = model_path / "tracker-meta.json";

  // Required detector engines + tokenizer.
  for (const auto &required :
       {vision_encoder_path, text_encoder_path, decoder_path, tokenizer_path}) {
    if (!std::filesystem::exists(required)) {
      reusex::error("TensorRTSam3p1: required file missing: {}", required);
      return nullptr;
    }
  }
  // Required tracker engines.
  if (!std::filesystem::exists(memory_encoder_path) ||
      !std::filesystem::exists(memory_attention_path)) {
    reusex::error("TensorRTSam3p1: tracker engines missing "
                  "(memory-encoder: {}, memory-attention: {}). Not a SAM 3.1 "
                  "video model directory.",
                  memory_encoder_path, memory_attention_path);
    return nullptr;
  }

  // geometry-encoder is optional. The video path uses text prompts only, so we
  // deliberately do NOT wire geometry here even when the engine is present.
  const std::string geom_path = "";

  const int gpu_id = 0; // Default GPU ID, can be parameterized.

  auto instance = std::make_unique<TensorRTSam3p1>(
      vision_encoder_path.string(), text_encoder_path.string(), geom_path,
      decoder_path.string(), memory_encoder_path.string(),
      memory_attention_path.string(), tokenizer_path.string(), meta_path,
      gpu_id);

  if (!instance->load_engines()) {
    reusex::error("Failed to load TensorRTSam3p1 engines from path: {}",
                  model_path);
    return nullptr;
  }

  reusex::info("TensorRTSam3p1 created successfully");
  return instance;
}

TensorRTSam3p1::TensorRTSam3p1(const std::string &vision_encoder_path,
                               const std::string &text_encoder_path,
                               const std::string &geometry_encoder_path,
                               const std::string &decoder_path,
                               const std::string &memory_encoder_path,
                               const std::string &memory_attention_path,
                               const std::string &tokenizer_path,
                               const std::filesystem::path &meta_path,
                               int gpu_id)
    : vision_encoder_path_(vision_encoder_path),
      text_encoder_path_(text_encoder_path),
      geometry_encoder_path_(geometry_encoder_path),
      decoder_path_(decoder_path), memory_encoder_path_(memory_encoder_path),
      memory_attention_path_(memory_attention_path), gpu_id_(gpu_id) {
  reusex::debug("Initializing TensorRTSam3p1 with gpu_id={}", gpu_id_);

  // Experimental: enable memory-attention feature-conditioning of the detector
  // via env var (off by default because it collapses the open-vocab detector —
  // see use_memory_conditioning_). Lets A/B testing without a rebuild.
  if (const char *e = std::getenv("REUSEX_SAM3P1_MEMORY_COND");
      e && std::string(e) != "0") {
    use_memory_conditioning_ = true;
    reusex::warn("SAM3.1 memory-conditioning ENABLED (experimental; degrades "
                 "the open-vocab detector)");
  }

  original_image_buf_ = std::make_shared<tensor::Memory<uint8_t>>();

  // Tokenizer (same pattern as Sam3.cpp).
  auto blob = load_bytes_from_file(tokenizer_path);
  reusex::debug("Tokenizer blob loaded, size: {} bytes", blob.size());
  tokenizer_ = tokenizers::Tokenizer::FromBlobJSON(blob);
  reusex::debug("Tokenizer initialized successfully with vocab size: {}",
                tokenizer_->GetVocabSize());

  // Load tracker-meta.json as fallback sizing. Engine-probed shapes always take
  // precedence in load_engines(); these are only used when the engine does not
  // report a static dimension.
  if (std::filesystem::exists(meta_path)) {
    try {
      std::ifstream fs(meta_path);
      nlohmann::json meta;
      fs >> meta;
      mem_bank_max_ = meta.value("mem_bank_max", mem_bank_max_);
      mem_tokens_per_frame_ =
          meta.value("mem_tokens_per_frame", mem_tokens_per_frame_);
      mem_dim_ = meta.value("mem_dim", mem_dim_);
      feat_c_ = meta.value("feat_c", feat_c_);
      feat_h_ = meta.value("feat_h", feat_h_);
      feat_w_ = meta.value("feat_w", feat_w_);
      multiplex_count_ = meta.value("multiplex_count", multiplex_count_);
      reusex::debug("Loaded tracker-meta.json: mem_bank_max={}, "
                    "mem_tokens_per_frame={}, mem_dim={}, feat={}x{}x{}, "
                    "multiplex_count={}",
                    mem_bank_max_, mem_tokens_per_frame_, mem_dim_, feat_c_,
                    feat_h_, feat_w_, multiplex_count_);
    } catch (const std::exception &e) {
      reusex::warn("Failed to parse tracker-meta.json ({}); relying on engine "
                   "probed shapes only",
                   e.what());
    }
  } else {
    reusex::debug(
        "No tracker-meta.json found; relying on engine probed shapes");
  }
}

std::string TensorRTSam3p1::load_bytes_from_file(const std::string &path) {
  // Identical to Sam3.cpp::load_bytes_from_file.
  std::ifstream fs(path, std::ios::in | std::ios::binary);
  if (fs.fail()) {
    reusex::error("Failed to open file at path: {}", path);
    exit(1);
  }
  std::string data;
  fs.seekg(0, std::ios::end);
  size_t size = static_cast<size_t>(fs.tellg());
  fs.seekg(0, std::ios::beg);
  data.resize(size);
  fs.read(data.data(), size);
  return data;
}

bool TensorRTSam3p1::load_engines() {
  reusex::info("Loading TensorRTSam3p1 engines");
  AutoDevice device_guard(gpu_id_);

  // Same loader lambda as Sam3.cpp::load_engines().
  auto load_engine = [&](const std::string &path,
                         std::shared_ptr<TensorRT::Engine> &engine,
                         const char *name) {
    if (path.empty()) {
      reusex::debug("Skipping {} engine (empty path)", name);
      return true;
    }
    reusex::debug("Loading {} engine from: {}", name, path);
    engine = TensorRT::load(path);
    if (!engine) {
      reusex::error("Failed to load {} from path: {}", name, path);
      return false;
    }
    engine->print(path.c_str());
    if (isdynamic_model_)
      isdynamic_model_ = engine->has_dynamic_dim();
    reusex::debug("{} engine loaded successfully", name);
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
    if (!load_engine(geometry_encoder_path_, geometry_encoder_trt_, "Geometry"))
      return false;
  }

  if (!load_engine(decoder_path_, decoder_trt_, "Decoder"))
    return false;
  auto pred_masks_shape = decoder_trt_->static_dims(6);
  auto pred_boxes_shape = decoder_trt_->static_dims(7);
  num_queries_ = pred_boxes_shape[1];
  mask_width_ = pred_masks_shape[2];
  mask_height_ = pred_masks_shape[3];

  if (!load_engine(memory_encoder_path_, memory_encoder_trt_, "MemoryEncoder"))
    return false;
  if (!load_engine(memory_attention_path_, memory_attention_trt_,
                   "MemoryAttention"))
    return false;

  // Verify the exported memory-encoder I/O contract matches what
  // append_memory() and build_aggregate_mask() assume (see
  // python/reusex_sam3/__init__.py). This catches export drift early instead of
  // at forward() time; dynamic axes report as <= 0 and are skipped. Non-fatal
  // (warn) so an updated export can still be exercised, but a mismatch means
  // the memory path would feed the engine wrong data.
  auto verify_binding = [&](TensorRT::Engine *eng, const char *tensor,
                            std::initializer_list<int> expect) {
    int idx = eng->index(tensor);
    if (idx < 0 || !eng->is_input(tensor)) {
      reusex::warn("memory-encoder: expected input binding '{}' not found",
                   tensor);
      return;
    }
    auto dims = eng->static_dims(tensor);
    if (dims.size() != expect.size()) {
      reusex::warn("memory-encoder '{}' rank {} != expected {}", tensor,
                   dims.size(), expect.size());
      return;
    }
    int axis = 0;
    for (int e : expect) {
      if (e > 0 && dims[axis] > 0 && dims[axis] != e)
        reusex::warn("memory-encoder '{}' axis {} = {} != expected {}", tensor,
                     axis, dims[axis], e);
      ++axis;
    }
  };
  // pred_mask [K,1,1008,1008] raw logits; vision_feat [1,C,H,W]; scores [K,1].
  // K (num_objects) is dynamic → pass 0 to skip that axis.
  verify_binding(memory_encoder_trt_.get(), "pred_mask",
                 {0, 1, input_image_height_, input_image_width_});
  verify_binding(memory_encoder_trt_.get(), "vision_feat", {1, 0, 0, 0});
  verify_binding(memory_encoder_trt_.get(), "object_score_logits", {0, 1});

  // Probe current-feat shape from the memory-attention output pix_feat_with_mem
  // [B, C=256, H=72, W=72] — conditioning is at the fpn_feat_2 (72x72) level.
  // Prefer engine-reported values over tracker-meta.json fallbacks. We use the
  // memory-encoder/attention spatial output (NOT fpn_feat_0_shape_, which is
  // the 288x288 level) so feat_h_/feat_w_ describe the 72x72 memory grid.
  {
    int me_out_idx = memory_encoder_trt_->index("maskmem_features");
    std::vector<int> mm;
    if (me_out_idx >= 0)
      mm = memory_encoder_trt_->static_dims(me_out_idx);
    // maskmem_features is spatial [B, C, H, W]. Derive feat dims from it.
    if (mm.size() == 4) {
      if (mm[1] > 0)
        feat_c_ = mm[1];
      if (mm[2] > 0)
        feat_h_ = mm[2];
      if (mm[3] > 0)
        feat_w_ = mm[3];
    } else {
      reusex::warn("memory-encoder 'maskmem_features' not a 4-D [B,C,H,W] "
                   "binding (rank={}); relying on tracker-meta.json feat dims",
                   mm.size());
    }
  }

  // A memory token is SPATIAL: mem_tokens_per_frame = H*W, mem_dim = C. This is
  // the seq-major [H*W, C] layout consumed by memory-attention. Derive from the
  // probed feature grid; tracker-meta.json only fills in dynamic (-1) dims.
  if (feat_h_ > 0 && feat_w_ > 0)
    mem_tokens_per_frame_ = feat_h_ * feat_w_;
  // mem_dim == channel dim by construction (seq-major [H*W, C]).
  if (feat_c_ > 0)
    mem_dim_ = feat_c_;

  // Cross-check against tracker-meta.json when both are available.
  if (mem_tokens_per_frame_ > 0 && feat_h_ > 0 && feat_w_ > 0 &&
      mem_tokens_per_frame_ != feat_h_ * feat_w_)
    reusex::warn("mem_tokens_per_frame ({}) != feat_h*feat_w ({}*{}); using "
                 "probed feature grid",
                 mem_tokens_per_frame_, feat_h_, feat_w_);

  if (mem_bank_max_ <= 0)
    mem_bank_max_ = 7;

  reusex::info("SAM 3.1 memory bank: mem_bank_max={}, mem_tokens_per_frame={}, "
               "mem_dim={}, current_feat={}x{}x{}",
               mem_bank_max_, mem_tokens_per_frame_, mem_dim_, feat_c_, feat_h_,
               feat_w_);

  allocate_memory_once();

  reusex::info("All TensorRTSam3p1 engines loaded successfully");
  return true;
}

void TensorRTSam3p1::allocate_memory_once() {
  reusex::debug("Allocating GPU memory for TensorRTSam3p1");

  // --- Image buffers (batch=1), sized as in Sam3.cpp::allocate_memory_once.
  // ---
  affine_matrix_.cpu(6);
  affine_matrix_.gpu(6);
  mask_affine_matrix_.cpu(6);
  mask_affine_matrix_.gpu(6);
  preprocessed_images_.gpu(3 * input_image_height_ * input_image_width_);

  size_t feat_0_sz_one = static_cast<size_t>(fpn_feat_0_shape_[1]) *
                         fpn_feat_0_shape_[2] * fpn_feat_0_shape_[3];
  fpn_feat_0_.gpu(feat_0_sz_one);
  fpn_feat_1_.gpu(feat_0_sz_one / 4);
  fpn_feat_2_.gpu(feat_0_sz_one / 16);
  fpn_pos_2_.gpu(feat_0_sz_one / 16);

  // --- Text prompt inputs/features (batch=1). ---
  size_t text_in_sz = static_cast<size_t>(text_ids_shape_[1]);
  text_input_ids_.cpu(text_in_sz);
  text_input_ids_.gpu(text_in_sz);
  text_attention_mask_.cpu(text_in_sz);
  text_attention_mask_.gpu(text_in_sz);
  text_features_.gpu(text_in_sz * 256);
  text_mask_.gpu(text_in_sz);

  // --- Decoder prompt inputs (text-only for the video path). ---
  size_t prompt_len = text_ids_shape_[1];
  prompt_features_.gpu(prompt_len * 256);
  prompt_mask_.gpu(prompt_len);

  // --- Decoder outputs (batch=1). ---
  pred_masks_.gpu(static_cast<size_t>(num_queries_) * mask_height_ *
                  mask_width_);
  pred_boxes_.gpu(static_cast<size_t>(num_queries_) * 4);
  pred_logits_.gpu(num_queries_);
  presence_logits_.gpu(1);

  // --- Postprocess buffers (batch=1). ---
  size_t post_sz = static_cast<size_t>(num_queries_);
  filter_boxes_.cpu(post_sz * 4);
  filter_boxes_.gpu(post_sz * 4);
  filter_scores_.cpu(post_sz);
  filter_scores_.gpu(post_sz);
  filter_indices_.cpu(post_sz);
  filter_indices_.gpu(post_sz);
  box_count_.cpu(1);
  box_count_.gpu(1);
  box_affine_matrices_.cpu(post_sz * 6);
  box_affine_matrices_.gpu(post_sz * 6);

  // --- Memory-bank buffers. ---
  // A memory frame is spatial [C=feat_c_, H=feat_h_, W=feat_w_]; as a sequence
  // of tokens it is [H*W, C] = [tokens, dim]. The element count is identical
  // (C*H*W == tokens*dim), only the LAYOUT differs: slots store seq-major
  // [tokens, dim], the memory-encoder emits spatial [C,H,W].
  size_t tokens = std::max(1, mem_tokens_per_frame_);
  size_t dim = std::max(1, mem_dim_);
  size_t per_frame = tokens * dim; // == feat_c_*feat_h_*feat_w_
  size_t spatial = static_cast<size_t>(std::max(1, feat_c_)) *
                   std::max(1, feat_h_) * std::max(1, feat_w_);

  // Memory-encoder inputs. The SimpleMaskEncoder is a MULTIPLEX encoder with a
  // fixed K=multiplex_count object-mask channels [K,1,input_h,input_w]
  // (1008x1008); we place the aggregate foreground mask in channel 0 and leave
  // channels 1..K-1 empty (background). object_score_logits is [K,1].
  size_t mask_px =
      static_cast<size_t>(input_image_height_) * input_image_width_;
  mem_pred_mask_.gpu(mask_px * multiplex_count_);
  mem_pred_mask_host_.cpu(mask_px * multiplex_count_);
  mem_obj_score_.cpu(multiplex_count_);
  mem_obj_score_.gpu(multiplex_count_);

  // Per-frame memory-encoder outputs, spatial [C,H,W].
  maskmem_features_.gpu(spatial);
  maskmem_pos_enc_.gpu(spatial);

  // Ring slots, stored seq-major [tokens, dim].
  mem_slots_.resize(mem_bank_max_);
  for (auto &slot : mem_slots_) {
    slot.feature.gpu(per_frame);
    slot.pos.gpu(per_frame);
  }

  // current_feat / current_pos for memory-attention: seq-major [H*W,1,C].
  current_feat_.gpu(per_frame);
  current_pos_.gpu(per_frame);

  // Host scratch for the [C,H,W] <-> [H*W,C] transpose (src + dst planes).
  transpose_scratch_.cpu(2 * spatial);

  // Packed memory + key-padding mask consumed by memory-attention. memory /
  // memory_pos are seq-major [M,1,C]; memory_mask is [1,M] (bool, all-true —
  // ignored by the RoPE encoder but bound for API completeness).
  mem_feat_concat_.gpu(static_cast<size_t>(mem_bank_max_) * per_frame);
  mem_pos_concat_.gpu(static_cast<size_t>(mem_bank_max_) * per_frame);
  memory_mask_.cpu(static_cast<size_t>(mem_bank_max_) * tokens);
  memory_mask_.gpu(static_cast<size_t>(mem_bank_max_) * tokens);

  // Memory-attention output (memory-conditioned current-frame embedding),
  // spatial [C,H,W] — replaces fpn_feat_2_.
  pix_feat_with_mem_.gpu(spatial);

  reusex::debug("GPU memory allocation completed for TensorRTSam3p1");
}

void TensorRTSam3p1::set_binding_dim(std::shared_ptr<TensorRT::Engine> &engine,
                                     int idx, const std::vector<int> &dims) {
  // Gate on the ENGINE's own dynamic flag, not the model-wide AND
  // (isdynamic_model_): the static tracker engines (memory-encoder is fixed at
  // K=16) must not suppress run-dim setting on the dynamic vision/text/decoder
  // engines, which would fail with "input shapes not specified".
  if (engine && engine->has_dynamic_dim())
    engine->set_run_dims(idx, dims);
}

void TensorRTSam3p1::reset() {
  reusex::debug("Resetting TensorRTSam3p1 memory bank");
  mem_valid_.clear();
  mem_next_slot_ = 0;
  frame_counter_ = 0;
}

// --- Detector stages (batch=1), modelled on Sam3.cpp. ------------------------

void TensorRTSam3p1::preprocess(const TensorRTData &input, void *stream) {
  // Mirrors Sam3.cpp::preprocess with ibatch fixed to 0.
  cudaStream_t s = (cudaStream_t)stream;
  const cv::Mat &img = input.image;
  common_tensor::Image img_tensor = common_tensor::cvimg(img);

  original_image_size_ = {img_tensor.width, img_tensor.height};

  affine::ResizeMatrix matrix;
  matrix.compute(std::make_tuple(img_tensor.width, img_tensor.height),
                 std::make_tuple(input_image_width_, input_image_height_));

  size_t size_image =
      static_cast<size_t>(img_tensor.width) * img_tensor.height * 3;
  uint8_t *h_buf = original_image_buf_->cpu(size_image);

  if (img.isContinuous()) {
    memcpy(h_buf, img.data, size_image);
  } else {
    int w_bytes = img_tensor.width * 3;
    for (int h = 0; h < img_tensor.height; ++h)
      memcpy(h_buf + h * w_bytes, img.ptr<uint8_t>(h), w_bytes);
  }

  float *h_mat = affine_matrix_.cpu();
  memcpy(h_mat, matrix.d2i, sizeof(matrix.d2i));

  cudaMemcpyAsync(original_image_buf_->gpu(size_image), h_buf, size_image,
                  cudaMemcpyHostToDevice, s);
  cudaMemcpyAsync(affine_matrix_.gpu(), h_mat, sizeof(matrix.d2i),
                  cudaMemcpyHostToDevice, s);

  affine::ResizeMatrix mask_m;
  mask_m.compute(std::make_tuple(mask_width_, mask_height_),
                 std::make_tuple(img_tensor.width, img_tensor.height));
  memcpy(mask_affine_matrix_.cpu(), mask_m.d2i, sizeof(mask_m.d2i));
  cudaMemcpyAsync(mask_affine_matrix_.gpu(), mask_m.d2i, sizeof(mask_m.d2i),
                  cudaMemcpyHostToDevice, s);

  warp_affine_bilinear_and_normalize_plane(
      original_image_buf_->gpu(), img_tensor.width * 3, img_tensor.width,
      img_tensor.height, preprocessed_images_.gpu(), input_image_width_,
      input_image_height_, affine_matrix_.gpu(), 114, preprocess_norm_, s);
}

bool TensorRTSam3p1::encode_image(void *stream) {
  // Mirrors Sam3.cpp::encode_image with batch_size fixed to 1.
  set_binding_dim(vision_encoder_trt_, 0,
                  {1, 3, input_image_height_, input_image_width_});

  bool success =
      vision_encoder_trt_->forward({{"images", preprocessed_images_.gpu()},
                                    {"fpn_feat_0", fpn_feat_0_.gpu()},
                                    {"fpn_feat_1", fpn_feat_1_.gpu()},
                                    {"fpn_feat_2", fpn_feat_2_.gpu()},
                                    {"fpn_pos_2", fpn_pos_2_.gpu()}},
                                   (cudaStream_t)stream);
  if (!success)
    reusex::error("Vision encoder forward pass failed");
  return success;
}

bool TensorRTSam3p1::encode_text(const Sam3PromptUnit *prompt, void *stream) {
  // Mirrors Sam3.cpp::encode_text, batch_size == 1.
  int seq_len = text_ids_shape_[1];
  int64_t *h_ids = text_input_ids_.cpu();
  int64_t *h_mask = text_attention_mask_.cpu();

  std::array<int64_t, 32> def_ids;
  def_ids.fill(49407);
  std::array<int64_t, 32> def_mask = {0};
  def_mask[0] = 1;

  const int64_t *src_ids = def_ids.data();
  const int64_t *src_mask = def_mask.data();
  if (prompt && text_input_map_.count(prompt->text)) {
    src_ids = std::get<0>(text_input_map_[prompt->text]).data();
    src_mask = std::get<1>(text_input_map_[prompt->text]).data();
  }
  memcpy(h_ids, src_ids, seq_len * sizeof(int64_t));
  memcpy(h_mask, src_mask, seq_len * sizeof(int64_t));

  cudaStream_t s = (cudaStream_t)stream;
  cudaMemcpyAsync(text_input_ids_.gpu(), h_ids, seq_len * sizeof(int64_t),
                  cudaMemcpyHostToDevice, s);
  cudaMemcpyAsync(text_attention_mask_.gpu(), h_mask, seq_len * sizeof(int64_t),
                  cudaMemcpyHostToDevice, s);

  set_binding_dim(text_encoder_trt_, 0, {1, seq_len});
  set_binding_dim(text_encoder_trt_, 1, {1, seq_len});

  return text_encoder_trt_->forward(
      {{"input_ids", text_input_ids_.gpu()},
       {"attention_mask", text_attention_mask_.gpu()},
       {"text_features", text_features_.gpu()},
       {"text_mask", text_mask_.gpu()}},
      s);
}

bool TensorRTSam3p1::decode(void *stream) {
  // Mirrors Sam3.cpp::decode, batch_size == 1, text prompts only (no geometry).
  int text_len = text_ids_shape_[1];
  int feat_dim = 256;
  size_t feat_sz = feat_dim * sizeof(float);
  size_t mask_sz = sizeof(bool);
  cudaStream_t s = (cudaStream_t)stream;

  // Prompt features == text features (no geometry tail on the video path).
  cudaMemcpyAsync(prompt_features_.gpu(), text_features_.gpu(),
                  text_len * feat_sz, cudaMemcpyDeviceToDevice, s);
  cudaMemcpyAsync(prompt_mask_.gpu(), text_mask_.gpu(), text_len * mask_sz,
                  cudaMemcpyDeviceToDevice, s);

  set_binding_dim(
      decoder_trt_, 0,
      {1, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2], fpn_feat_0_shape_[3]});
  set_binding_dim(decoder_trt_, 1,
                  {1, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 2,
                   fpn_feat_0_shape_[3] / 2});
  set_binding_dim(decoder_trt_, 2,
                  {1, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 4,
                   fpn_feat_0_shape_[3] / 4});
  set_binding_dim(decoder_trt_, 3,
                  {1, fpn_feat_0_shape_[1], fpn_feat_0_shape_[2] / 4,
                   fpn_feat_0_shape_[3] / 4});
  set_binding_dim(decoder_trt_, 4, {1, text_len, 256});
  set_binding_dim(decoder_trt_, 5, {1, text_len});

  // fpn_feat_2_ carries the memory-conditioned embedding when the bank is
  // non-empty (see apply_memory_attention()); fpn_feat_0/1 + fpn_pos_2 are raw.
  return decoder_trt_->forward({{"fpn_feat_0", fpn_feat_0_.gpu()},
                                {"fpn_feat_1", fpn_feat_1_.gpu()},
                                {"fpn_feat_2", fpn_feat_2_.gpu()},
                                {"fpn_pos_2", fpn_pos_2_.gpu()},
                                {"prompt_features", prompt_features_.gpu()},
                                {"prompt_mask", prompt_mask_.gpu()},
                                {"pred_masks", pred_masks_.gpu()},
                                {"pred_boxes", pred_boxes_.gpu()},
                                {"pred_logits", pred_logits_.gpu()},
                                {"presence_logits", presence_logits_.gpu()}},
                               s);
}

void TensorRTSam3p1::postprocess(InferResult &image_result,
                                 const std::string &label, int label_id,
                                 float confidence_threshold, void *stream) {
  // Mirrors Sam3.cpp::postprocess with batch_idx/image_idx fixed to 0.
  cudaStream_t s = (cudaStream_t)stream;

  float *d_pred_masks = pred_masks_.gpu();
  float *d_pred_boxes = pred_boxes_.gpu();
  float *d_pred_logits = pred_logits_.gpu();
  float *d_presence = presence_logits_.gpu();

  float *d_filter_boxes = filter_boxes_.gpu();
  float *d_filter_scores = filter_scores_.gpu();
  int *d_filter_indices = filter_indices_.gpu();

  cudaMemsetAsync(box_count_.gpu(), 0, sizeof(int), s);

  sam3_postprocess_plane(d_pred_masks, d_pred_boxes, d_pred_logits, d_presence,
                         d_filter_boxes, d_filter_indices, d_filter_scores,
                         box_count_.gpu(), num_queries_, mask_height_,
                         mask_width_, original_image_size_.first,
                         original_image_size_.second, confidence_threshold, s);

  cudaMemcpyAsync(box_count_.cpu(), box_count_.gpu(), sizeof(int),
                  cudaMemcpyDeviceToHost, s);
  cudaStreamSynchronize(s);
  int count = *box_count_.cpu();
  if (count <= 0)
    return;

  std::vector<float> h_boxes(count * 4);
  std::vector<float> h_scores(count);
  std::vector<int> h_indices(count);

  cudaMemcpyAsync(h_boxes.data(), d_filter_boxes, count * 4 * sizeof(float),
                  cudaMemcpyDeviceToHost, s);
  cudaMemcpyAsync(h_scores.data(), d_filter_scores, count * sizeof(float),
                  cudaMemcpyDeviceToHost, s);
  cudaMemcpyAsync(h_indices.data(), d_filter_indices, count * sizeof(int),
                  cudaMemcpyDeviceToHost, s);

  float *h_base_matrix = mask_affine_matrix_.cpu();
  float *h_box_matrices = box_affine_matrices_.cpu();

  size_t total_mask_pixels = 0;
  std::vector<size_t> mask_offsets(count);
  std::vector<cv::Size> mask_sizes(count);

  for (int i = 0; i < count; ++i) {
    float *b = h_boxes.data() + i * 4;
    int x1 = std::max(0, (int)b[0]);
    int y1 = std::max(0, (int)b[1]);
    int x2 = std::min(original_image_size_.first, (int)b[2]);
    int y2 = std::min(original_image_size_.second, (int)b[3]);

    int box_w = std::max(1, x2 - x1);
    int box_h = std::max(1, y2 - y1);

    mask_sizes[i] = cv::Size(box_w, box_h);
    mask_offsets[i] = total_mask_pixels;
    total_mask_pixels += static_cast<size_t>(box_w) * box_h;

    float *m_dst = h_box_matrices + i * 6;
    m_dst[0] = h_base_matrix[0];
    m_dst[1] = h_base_matrix[1];
    m_dst[3] = h_base_matrix[3];
    m_dst[4] = h_base_matrix[4];
    m_dst[2] = h_base_matrix[0] * x1 + h_base_matrix[1] * y1 + h_base_matrix[2];
    m_dst[5] = h_base_matrix[3] * x1 + h_base_matrix[4] * y1 + h_base_matrix[5];
  }

  mask_buffer_.gpu(total_mask_pixels);
  mask_buffer_.cpu(total_mask_pixels);

  cudaMemcpyAsync(box_affine_matrices_.gpu(), box_affine_matrices_.cpu(),
                  count * 6 * sizeof(float), cudaMemcpyHostToDevice, s);

  for (int i = 0; i < count; ++i) {
    int idx = h_indices[i];
    float *src = pred_masks_.gpu() + idx * mask_height_ * mask_width_;
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
    image_result.push_back(object::create_segmentation_box(
        b[0], b[1], b[2], b[3], bin_mask.clone(), h_scores[i], label_id,
        label));
  }
}

// --- Memory bank -------------------------------------------------------------

void TensorRTSam3p1::rearrange_chw_to_hwc(float *d_src, float *d_dst, int c,
                                          int h, int w, void *stream) {
  // Convert spatial [C,H,W] (index c*H*W + h*W + w) to seq-major [H*W,C]
  // (index (h*W+w)*C + c). No transpose kernel exists in this module and the
  // task forbids adding one, so we do a host round-trip: D2H the source plane,
  // transpose on the CPU into a second host plane, then H2D into the dst.
  cudaStream_t s = (cudaStream_t)stream;
  const size_t hw = static_cast<size_t>(h) * w;
  const size_t n = static_cast<size_t>(c) * hw;

  float *h_src = transpose_scratch_.cpu(); // [0, n)  = source [C,HW]
  float *h_dst = h_src + n;                // [n, 2n) = dest   [HW,C]

  cudaMemcpyAsync(h_src, d_src, n * sizeof(float), cudaMemcpyDeviceToHost, s);
  cudaStreamSynchronize(s); // need the data on host before transposing

  for (int ci = 0; ci < c; ++ci) {
    const float *row = h_src + static_cast<size_t>(ci) * hw; // channel ci
    for (size_t p = 0; p < hw; ++p)
      h_dst[p * c + ci] = row[p];
  }

  cudaMemcpyAsync(d_dst, h_dst, n * sizeof(float), cudaMemcpyHostToDevice, s);
}

int TensorRTSam3p1::pack_memory(void *stream) {
  cudaStream_t s = (cudaStream_t)stream;
  const size_t tokens = std::max(1, mem_tokens_per_frame_);
  const size_t dim = std::max(1, mem_dim_);
  const size_t per_frame = tokens * dim; // seq-major [tokens, dim] per slot

  int packed = 0;
  // Slots are already stored seq-major [tokens, dim] (see append_memory), so
  // concatenating M valid slots front-first yields the memory tensor
  // [M*tokens, 1, dim] with plain contiguous D2D copies — no rearrange here.
  // Same D2D gather idiom as Sam3.cpp::gather_vision_features.
  bool *h_mask = memory_mask_.cpu();
  memset(h_mask, 0, memory_mask_.cpu_bytes());

  for (int slot : mem_valid_) {
    float *dst_f = mem_feat_concat_.gpu() + packed * per_frame;
    float *dst_p = mem_pos_concat_.gpu() + packed * per_frame;
    cudaMemcpyAsync(dst_f, mem_slots_[slot].feature.gpu(),
                    per_frame * sizeof(float), cudaMemcpyDeviceToDevice, s);
    cudaMemcpyAsync(dst_p, mem_slots_[slot].pos.gpu(),
                    per_frame * sizeof(float), cudaMemcpyDeviceToDevice, s);
    for (size_t t = 0; t < tokens; ++t)
      h_mask[packed * tokens + t] = true; // memory_mask_ = [1, M] all-true
    ++packed;
  }

  cudaMemcpyAsync(memory_mask_.gpu(), h_mask,
                  static_cast<size_t>(packed) * tokens * sizeof(bool),
                  cudaMemcpyHostToDevice, s);

  // Total valid memory tokens M = num_valid_slots * tokens.
  return packed * static_cast<int>(tokens);
}

bool TensorRTSam3p1::apply_memory_attention(void *stream) {
  cudaStream_t s = (cudaStream_t)stream;
  if (mem_valid_.empty())
    return true; // No memory yet: keep raw fpn_feat_2_.

  // Contract (tracker-memory-attention):
  //   current_feat [5184,1,256] = fpn_feat_2 permuted [C,H,W]->[H*W,C] + batch
  //   current_pos  [5184,1,256] = fpn_pos_2  permuted [C,H,W]->[H*W,C] + batch
  //   memory       [M,1,256]    = packed valid slots (seq-major)
  //   memory_pos   [M,1,256]    = packed valid slot pos-encodings
  //   memory_mask  [1,M]        = all-true (RoPE encoder ignores it)
  //   output pix_feat_with_mem [1,256,72,72] REPLACES fpn_feat_2 for decode.
  //
  // Conditioning is at the fpn_feat_2 (72x72) level, NOT fpn_feat_0. Pack the
  // valid memory FIRST (fills memory_mask_) then rearrange the current feats.
  int valid_tokens = pack_memory(stream); // M = num_valid_slots * H*W

  // fpn_feat_2_ / fpn_pos_2_ are spatial [1,C,H,W]; rearrange to seq-major
  // [H*W,C] (== [H*W,1,C] for B=1) for the attention token inputs.
  rearrange_chw_to_hwc(fpn_feat_2_.gpu(), current_feat_.gpu(), feat_c_, feat_h_,
                       feat_w_, stream);
  rearrange_chw_to_hwc(fpn_pos_2_.gpu(), current_pos_.gpu(), feat_c_, feat_h_,
                       feat_w_, stream);

  const int hw = feat_h_ * feat_w_; // 5184

  set_binding_dim(memory_attention_trt_, 0, {hw, 1, mem_dim_}); // current_feat
  set_binding_dim(memory_attention_trt_, 1, {hw, 1, mem_dim_}); // current_pos
  set_binding_dim(memory_attention_trt_, 2,
                  {valid_tokens, 1, mem_dim_}); // memory  [M,1,C]
  set_binding_dim(memory_attention_trt_, 3,
                  {valid_tokens, 1, mem_dim_}); // memory_pos [M,1,C]
  set_binding_dim(memory_attention_trt_, 4, {1, valid_tokens}); // memory_mask

  bool ok = memory_attention_trt_->forward(
      {{"current_feat", current_feat_.gpu()},
       {"current_pos", current_pos_.gpu()},
       {"memory", mem_feat_concat_.gpu()},
       {"memory_pos", mem_pos_concat_.gpu()},
       {"memory_mask", memory_mask_.gpu()},
       {"pix_feat_with_mem", pix_feat_with_mem_.gpu()}},
      s);
  if (!ok) {
    reusex::error("Memory-attention forward pass failed");
    return false;
  }

  // pix_feat_with_mem is spatial [1,C,H,W]. Overwriting fpn_feat_2_ with it
  // lets the decoder consume the conditioned feature — but this collapses the
  // open-vocab detector (see use_memory_conditioning_), so it is gated off by
  // default. When off we keep the raw fpn_feat_2_ for detection; the memory
  // bank is still updated so future detect-then-associate work can use it.
  if (use_memory_conditioning_) {
    size_t spatial = static_cast<size_t>(feat_c_) * feat_h_ * feat_w_;
    cudaMemcpyAsync(fpn_feat_2_.gpu(), pix_feat_with_mem_.gpu(),
                    spatial * sizeof(float), cudaMemcpyDeviceToDevice, s);
  }
  return true;
}

void TensorRTSam3p1::build_aggregate_mask(const InferResult &results,
                                          void *stream) {
  cudaStream_t s = (cudaStream_t)stream;
  const int H = input_image_height_; // 1008
  const int W = input_image_width_;  // 1008

  // VERIFIED against the exported engine
  // (python/reusex_sam3/wrappers_tracker.py MemoryEncoderWrapper.forward,
  // contract in python/reusex_sam3/__init__.py): the `pred_mask` binding is RAW
  // pre-sigmoid logits [K,1,1008,1008,f32] — the encoder applies mask_for_mem =
  // sigmoid(pred_mask)*scale+bias (scale=2.0, bias=-1.0) INSIDE the engine
  // graph. So we only need to supply logits that saturate the sigmoid: we build
  // a binary foreground union from the detector's per-object masks (postprocess
  // has already thresholded at confidence_threshold) and map
  // foreground->+kLogit / background->-kLogit. sigmoid(+/-10) ~= 1/0, which the
  // internal *2-1 maps to ~[-1,+1]; the exact scale/bias never has to be
  // replicated here. This is an aggregate (open-vocabulary, all-objects)
  // foreground mask, not a per-object mask; see the step() note on the
  // per-object-vs-aggregate design choice.
  constexpr float kLogit = 10.0f;

  // Original-image -> 1008x1008 mapping is the vision-encoder ResizeMatrix, a
  // per-axis stretch (see affine::ResizeMatrix): px = x * W/orig_w,
  // py = y * H/orig_h. Reuse the recorded original size.
  const int orig_w = std::max(1, original_image_size_.first);
  const int orig_h = std::max(1, original_image_size_.second);
  const float sx = static_cast<float>(W) / orig_w;
  const float sy = static_cast<float>(H) / orig_h;

  // Build a binary foreground canvas at 1008x1008 on the host.
  cv::Mat fg(H, W, CV_8U, cv::Scalar(0));
  for (const auto &det : results) {
    if (!det.segmentation.has_value() || det.segmentation->mask.empty())
      continue;
    const cv::Mat &m = det.segmentation->mask; // CV_8U at box resolution

    // Destination box in the 1008x1008 frame.
    int dx1 =
        std::clamp(static_cast<int>(std::lround(det.box.left * sx)), 0, W);
    int dy1 = std::clamp(static_cast<int>(std::lround(det.box.top * sy)), 0, H);
    int dx2 =
        std::clamp(static_cast<int>(std::lround(det.box.right * sx)), 0, W);
    int dy2 =
        std::clamp(static_cast<int>(std::lround(det.box.bottom * sy)), 0, H);
    int bw = dx2 - dx1;
    int bh = dy2 - dy1;
    if (bw <= 0 || bh <= 0)
      continue;

    cv::Mat resized;
    cv::resize(m, resized, cv::Size(bw, bh), 0, 0, cv::INTER_NEAREST);
    cv::Mat roi = fg(cv::Rect(dx1, dy1, bw, bh));
    cv::bitwise_or(roi, resized, roi); // union of object masks
  }

  // Convert the binary union to logits into the host staging buffer, then H2D.
  // Channel 0 carries the aggregate foreground; channels 1..K-1 are all
  // background (empty object slots) so the multiplex encoder sees one object.
  float *h_mask = mem_pred_mask_host_.cpu();
  const size_t px = static_cast<size_t>(H) * W;
  const uint8_t *fgp = fg.ptr<uint8_t>(0);
  for (size_t i = 0; i < px; ++i)
    h_mask[i] = fgp[i] ? kLogit : -kLogit;
  for (int c = 1; c < multiplex_count_; ++c)
    std::fill_n(h_mask + static_cast<size_t>(c) * px, px, -kLogit);

  cudaMemcpyAsync(mem_pred_mask_.gpu(), h_mask,
                  static_cast<size_t>(multiplex_count_) * px * sizeof(float),
                  cudaMemcpyHostToDevice, s);
}

void TensorRTSam3p1::append_memory(float object_score_logits, void *stream) {
  cudaStream_t s = (cudaStream_t)stream;

  // Upload object score logits [K,1]: channel 0 = this frame's max detection
  // score, channels 1..K-1 = strongly-negative (empty object slots).
  float *h_score = mem_obj_score_.cpu();
  h_score[0] = object_score_logits;
  for (int c = 1; c < multiplex_count_; ++c)
    h_score[c] = -10.0f;
  cudaMemcpyAsync(mem_obj_score_.gpu(), h_score,
                  static_cast<size_t>(multiplex_count_) * sizeof(float),
                  cudaMemcpyHostToDevice, s);

  // Contract (tracker-memory-encoder):
  //   vision_feat         = current top-level feature [B=1,C=256,H=72,W=72].
  //                         We pass fpn_feat_2_, which apply_memory_attention()
  //                         has overwritten in place with the
  //                         memory-conditioned embedding when the bank was
  //                         non-empty (raw fpn_feat_2 on the first frame). This
  //                         mirrors SAM2's _encode_new_memory, which encodes
  //                         memory from the conditioned pix_feat.
  //   pred_mask           = aggregate foreground logits
  //                         [K=multiplex_count,1,1008,1008] (fg in channel 0)
  //   object_score_logits = [K=multiplex_count,1] (channel 0 = max detection
  //                         score this frame; channels 1..K-1 strongly
  //                         negative)
  // Outputs maskmem_features / maskmem_pos_enc are spatial [B,256,72,72].
  set_binding_dim(memory_encoder_trt_, 0,
                  {1, feat_c_, feat_h_, feat_w_}); // vision_feat
  set_binding_dim(memory_encoder_trt_, 1,
                  {multiplex_count_, 1, input_image_height_,
                   input_image_width_}); // pred_mask
  set_binding_dim(memory_encoder_trt_, 2,
                  {multiplex_count_, 1}); // object_score_logits

  bool ok = memory_encoder_trt_->forward(
      {{"vision_feat", fpn_feat_2_.gpu()},
       {"pred_mask", mem_pred_mask_.gpu()},
       {"object_score_logits", mem_obj_score_.gpu()},
       {"maskmem_features", maskmem_features_.gpu()},
       {"maskmem_pos_enc", maskmem_pos_enc_.gpu()}},
      s);
  if (!ok) {
    reusex::error("Memory-encoder forward pass failed; frame not appended");
    return;
  }

  // Rotate into the ring: (re)use mem_next_slot_, evicting the oldest if full.
  // The encoder outputs are spatial [C,H,W]; rearrange to seq-major [H*W,C]
  // before storing so pack_memory() stays a plain contiguous copy.
  int slot = mem_next_slot_;
  rearrange_chw_to_hwc(maskmem_features_.gpu(), mem_slots_[slot].feature.gpu(),
                       feat_c_, feat_h_, feat_w_, stream);
  rearrange_chw_to_hwc(maskmem_pos_enc_.gpu(), mem_slots_[slot].pos.gpu(),
                       feat_c_, feat_h_, feat_w_, stream);

  if (static_cast<int>(mem_valid_.size()) >= mem_bank_max_)
    mem_valid_.pop_front(); // evict oldest
  mem_valid_.push_back(slot);
  mem_next_slot_ = (mem_next_slot_ + 1) % mem_bank_max_;

  cudaStreamSynchronize(s);
}

// --- IVideoModel::step -------------------------------------------------------

IDataset::Pair TensorRTSam3p1::step(const IDataset::Pair &in) {
  const TensorRTData *frame =
      dynamic_cast<const TensorRTData *>(in.first.get());
  if (!frame) {
    reusex::error("TensorRTSam3p1::step received non-TensorRTData input");
    IDataset::Pair out;
    out.first = std::make_unique<TensorRTData>();
    out.second = in.second;
    return out;
  }

  // Tokenize any unseen prompts (same cache idiom as Sam3.cpp::forward).
  for (const auto &prompt : frame->prompts) {
    if (text_input_map_.count(prompt.text) != 0)
      continue;
    auto [ids, mask] = make_ids(tokenizer_->Encode(prompt.text));
    int idx = (int)text_input_map_.size();
    text_input_map_[prompt.text] = std::make_tuple(ids, mask, idx);
    reusex::trace("Tokenized text: '{}' ({})", prompt.text, idx);
  }

  cudaStream_t stream = nullptr;
  if (auto s = cudaStreamCreate(&stream); s != cudaSuccess)
    reusex::error("Failed to create CUDA stream: {}", cudaGetErrorString(s));

  AutoDevice device_guard(gpu_id_);

  // (1) Vision encoder → raw fpn feats (batch=1).
  preprocess(*frame, stream);
  if (!encode_image(stream)) {
    reusex::error("Vision encoder failed at frame {}", frame_counter_);
    cudaStreamDestroy(stream);
    IDataset::Pair out;
    out.first = std::make_unique<TensorRTData>();
    out.second = in.second;
    return out;
  }

  // (2) Memory-attention: condition fpn_feat_2_ (the 72x72 top level) on the
  // memory bank (no-op on the first frame). fpn_feat_2_ is overwritten in place
  // with the memory-conditioned embedding on success; fpn_feat_0/1 + fpn_pos_2
  // pass through to the decoder unchanged.
  // Skipped entirely unless conditioning is enabled — the memory-attention
  // forward + its host-side transpose add ~50ms/frame of pure overhead when the
  // (unused, off-by-default) conditioning is disabled.
  if (use_memory_conditioning_)
    apply_memory_attention(stream);

  // (3) Detector (text-encode + decode + postprocess) on the conditioned feats.
  // The detector runs EVERY frame for open-vocabulary all-classes annotation;
  // the memory bank only stabilises the features across time.
  InferResult results;
  float best_score = 0.0f;
  for (const auto &prompt : frame->prompts) {
    std::string label =
        prompt.text.empty() ? std::string("object") : prompt.text;
    const int label_id = std::get<2>(text_input_map_[label]);

    if (!encode_text(&prompt, stream))
      continue;
    if (!decode(stream))
      continue;

    // Per-prompt threshold overrides the frame/global one when set (>= 0).
    const float conf = prompt.confidence >= 0.0f ? prompt.confidence
                                                 : frame->confidence_threshold;
    size_t before = results.size();
    postprocess(results, label, label_id, conf, stream);
    // Track the best detection score across concepts for object_score_logits.
    for (size_t i = before; i < results.size(); ++i)
      best_score = std::max(best_score, results[i].score);
  }

  // (4) Memory encoder (EXPERIMENTAL — only runs when memory-conditioning is
  // enabled). Builds this frame's aggregate foreground mask in channel 0 of the
  // multiplex_count-wide pred_mask (the rest empty) and appends the encoded
  // maskmem to the ring. NOTE: the memory-conditioning path is off by default
  // and empirically collapses the open-vocab detector — it feeds the SAM2-style
  // tracker's memory into the DETR detector's features, which is the wrong head
  // (see use_memory_conditioning_ / apply_memory_attention). It is retained
  // only for experimentation and would be replaced by a proper temporal design
  // (detect-then-associate). Skipping it entirely when off also avoids wasted
  // work.
  //
  // Design note (aggregate vs per-object): the multiplex memory encoder accepts
  // K=multiplex_count per-object mask channels, but this open-vocab path has no
  // stable per-object identity across frames, so we encode a single AGGREGATE
  // foreground (all detections unioned into channel 0, channels 1..K-1 left
  // background). A true per-object ring (one live mask per channel) would need
  // a detect-then-associate tracker to assign identities before encoding; that
  // is the same redesign that would revive memory-conditioning, so it is
  // intentionally deferred rather than approximated here.
  if (use_memory_conditioning_) {
    build_aggregate_mask(results, stream);
    append_memory(best_score, stream);
  }

  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);

  // Build the CV_32S label image (identical convention to Sam3.cpp::forward).
  auto res_ptr = std::make_unique<TensorRTData>();
  res_ptr->image = cv::Mat(frame->image.size(), CV_32S, cv::Scalar(-1));
  reusex::vision::osd::make_labled_image(res_ptr->image, results);

  IDataset::Pair out;
  out.first = std::move(res_ptr);
  out.second = in.second;

  ++frame_counter_;
  return out;
}

} // namespace reusex::vision::tensor_rt

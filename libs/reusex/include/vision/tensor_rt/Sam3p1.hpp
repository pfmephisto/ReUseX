// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/IVideoModel.hpp"
#include "reusex/vision/common/object.hpp"
#include "reusex/vision/tensor_rt/Data.hpp"
#include "reusex/vision/tensor_rt/Sam3Type.hpp"
#include "reusex/vision/tensor_rt/common/memory.hpp"
#include "reusex/vision/tensor_rt/common/norm.hpp"
#include "reusex/vision/tensor_rt/common/tensorrt.hpp"

#include <tokenizers_cpp.h>

#include <deque>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace reusex::vision::tensor_rt {

/* TensorRTSam3p1 implements the SAM 3.1 stateful video-tracker path on top of
 * TensorRT. It COEXISTS with TensorRTSam3 (the stateless SAM 3 detector) and is
 * selected when the model directory additionally contains the tracker engines
 * (tracker-memory-encoder.engine + tracker-memory-attention.engine).
 *
 * The per-frame detector path (vision-encoder + text-encoder + optional
 * geometry-encoder + decoder) is modelled CLOSELY on Sam3.cpp, but fixed to a
 * batch size of 1 and adapted to consume a memory-conditioned image embedding.
 * TensorRTSam3's members are private, so rather than sub-classing we replicate
 * the minimal batch=1 detector path here (see comments citing Sam3.cpp).
 *
 * On top of the detector, TensorRTSam3p1 maintains a fixed-size memory bank:
 * every frame's decoder output is fed through tracker-memory-encoder to produce
 * a maskmem token, which is appended to a ring of pre-allocated slots (oldest
 * evicted). Before running the detector on a new frame, the valid memory slots
 * are packed contiguously and fed — together with the current frame features —
 * through tracker-memory-attention to produce a memory-conditioned embedding
 * that stabilises the detector features across time.
 *
 * NOTE: this class implements IVideoModel (stateful) NOT IModel (stateless). It
 * MUST be driven by an ordered single-threaded loop (see IVideoModel contract),
 * never from the shuffled Dataloader. */
class TensorRTSam3p1 : public IVideoModel {
    private:
  using InferResult = ::reusex::vision::common::object::DetectionBoxArray;

    public:
  /* Constructor. Mirrors TensorRTSam3's constructor (Sam3.cpp) but additionally
   * takes the two tracker engine paths. Engine paths may be empty to indicate an
   * absent (optional) engine, matching the Sam3.cpp convention for geometry.
   */
  TensorRTSam3p1(const std::string &vision_encoder_path,
                 const std::string &text_encoder_path,
                 const std::string &geometry_encoder_path,
                 const std::string &decoder_path,
                 const std::string &memory_encoder_path,
                 const std::string &memory_attention_path,
                 const std::string &tokenizer_path,
                 const std::filesystem::path &meta_path, int gpu_id);

  /* Factory. Discovers the 4 detector engines + tokenizer.json + the 2 tracker
   * engines (memory-encoder, memory-attention) + optional tracker-meta.json in
   * model_path. Returns nullptr if any REQUIRED engine is missing (vision, text,
   * decoder, memory-encoder, memory-attention are required; geometry optional).
   * @param model_path: Directory containing the SAM 3.1 engine set.
   * @return A unique pointer to a TensorRTSam3p1, or nullptr on failure.
   */
  static std::unique_ptr<TensorRTSam3p1>
  create(const std::filesystem::path &model_path);

  /* IVideoModel::reset — clears the memory bank and zeroes the frame counter.
   * Call at every sequence boundary. */
  void reset() override;

  /* IVideoModel::step — process a single frame in temporal order. See the
   * IVideoModel contract for the input/output semantics. */
  IDataset::Pair step(const IDataset::Pair &in) override;

    protected:
  /* Loads all TensorRT engines and probes their static shapes (following the
   * Sam3.cpp load_engines() pattern), then sizes GPU memory. Returns true on
   * success. */
  bool load_engines();

  static std::string load_bytes_from_file(const std::string &file_path);

    private:
  // --- Detector stages (batch=1), modelled on Sam3.cpp -----------------------

  // Preprocess the single input frame into preprocessed_images_ (Sam3.cpp
  // preprocess()). ibatch is always 0 here.
  void preprocess(const TensorRTData &input, void *stream);

  // Run vision-encoder → fpn_feat_0/1/2 + fpn_pos_2 (Sam3.cpp encode_image()).
  bool encode_image(void *stream);

  // Run text-encoder for a single concept prompt (Sam3.cpp encode_text(),
  // simplified to batch=1). Uses the tokenizer cache text_input_map_.
  bool encode_text(const Sam3PromptUnit *prompt, void *stream);

  // Run decoder on the (memory-conditioned) fpn features (Sam3.cpp decode(),
  // batch=1, geometry omitted for the video path — text prompts only).
  bool decode(void *stream);

  // Postprocess decoder outputs into DetectionBoxArray (Sam3.cpp postprocess()).
  void postprocess(InferResult &image_result, const std::string &label,
                   int label_id, float confidence_threshold, void *stream);

  // --- Memory bank -----------------------------------------------------------

  // Run tracker-memory-encoder(vision_feat, pred_mask, object_score_logits) →
  // maskmem_features + maskmem_pos_enc, then append into the ring, evicting the
  // oldest slot. object_score_logits is derived from the fused detection scores.
  // The encoder outputs are spatial [C,H,W]; before storing them in a ring slot
  // they are rearranged to seq-major [H*W,C] so that pack_memory() is a plain
  // contiguous copy (see rearrange_chw_to_hwc()).
  void append_memory(float object_score_logits, void *stream);

  // Pack the currently-valid memory slots contiguously into mem_feat_concat_ /
  // mem_pos_concat_ and fill memory_mask_ (true for valid rows). Slots are
  // stored already in seq-major [H*W,C], so packing M slots yields the
  // [M*H*W, 1, C] `memory` tensor via straight D2D copies. Returns the number of
  // valid memory tokens packed (M * mem_tokens_per_frame_; 0 if bank is empty).
  int pack_memory(void *stream);

  // Run tracker-memory-attention(current_feat, current_pos, memory, memory_pos,
  // memory_mask) → pix_feat_with_mem, overwriting fpn_feat_2_ in-place with the
  // memory-conditioned embedding (conditioning happens at the 72x72 level =
  // fpn_feat_2, NOT fpn_feat_0). No-op when the bank is empty.
  bool apply_memory_attention(void *stream);

  // Build the aggregate foreground mask [1,1,input_h,input_w] fed to the memory
  // encoder from this frame's per-object detection masks: paste each object's
  // (already confidence-thresholded) binary mask into the 1008x1008 tracker
  // input frame (union), then convert to logits. Uploads into mem_pred_mask_.
  void build_aggregate_mask(const InferResult &results, void *stream);

  // Rearrange a device buffer from spatial [C,H,W] (row-major, the vision-encoder
  // / memory-encoder layout) to seq-major [H*W,C] (the memory-attention token
  // layout). Implemented as a host round-trip transpose (D2H → CPU transpose →
  // H2D) because no transpose kernel exists in this module and the task forbids
  // adding one; the element count (C*H*W ≈ 1.3M floats) makes this cheap enough
  // for the per-frame memory path. Uses transpose_scratch_ as staging.
  void rearrange_chw_to_hwc(float *d_src, float *d_dst, int c, int h, int w,
                            void *stream);

  // Allocate all fixed GPU/CPU buffers once, sized from probed engine shapes
  // (Sam3.cpp allocate_memory_once()).
  void allocate_memory_once();

  // set_run_dims wrapper for dynamic engines (Sam3.cpp set_binding_dim()).
  void set_binding_dim(std::shared_ptr<TensorRT::Engine> &engine,
                       int binding_index, const std::vector<int> &dims);

    private:
  // Configuration
  bool isdynamic_model_ = true;
  int input_image_width_ = 1008;
  int input_image_height_ = 1008;
  int gpu_id_ = 0;

  // Detector state (batch fixed to 1)
  std::pair<int, int> original_image_size_ = {0, 0};
  int num_queries_ = 200;
  int mask_height_ = 288;
  int mask_width_ = 288;

  // Memory-bank sizing. Probed from the tracker engines where possible; the
  // tracker-meta.json values are used only as fallback / cross-check.
  // A memory token is SPATIAL: each stored frame is a [C=256,H=72,W=72] feature
  // grid, i.e. mem_tokens_per_frame_ = feat_h_*feat_w_ = 5184 tokens of dim
  // mem_dim_ = feat_c_ = 256. Conditioning is at the fpn_feat_2 (72x72) level.
  int mem_bank_max_ = 7;         // number of frames retained in the ring
  int mem_tokens_per_frame_ = 0; // H*W spatial tokens per stored memory frame
  int mem_dim_ = 0;              // channel dim of each memory token (== feat_c_)
  int feat_c_ = 0;               // current-feat channel dim (fpn_feat_2 == 256)
  int feat_h_ = 0;               // current-feat height (72)
  int feat_w_ = 0;               // current-feat width  (72)
  int multiplex_count_ = 1;      // reserved (SAM 3.1 multiplex heads)
  int frame_counter_ = 0;

  // Model paths
  std::string vision_encoder_path_;
  std::string text_encoder_path_;
  std::string geometry_encoder_path_;
  std::string decoder_path_;
  std::string memory_encoder_path_;
  std::string memory_attention_path_;

  // TRT engines
  std::shared_ptr<TensorRT::Engine> vision_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> text_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> decoder_trt_;
  std::shared_ptr<TensorRT::Engine> geometry_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> memory_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> memory_attention_trt_;

  // Tokenizer cache: text -> (input_ids, attention_mask, class id). Same layout
  // as Sam3.cpp text_input_map_.
  std::unordered_map<std::string, std::tuple<std::array<int64_t, 32>,
                                             std::array<int64_t, 32>, int>>
      text_input_map_;

  // Preprocess normalisation (identical to Sam3.cpp).
  norm_image::Norm preprocess_norm_ = norm_image::Norm::alpha_beta(
      1.0f / 127.5f, -1.0f, norm_image::ChannelType::SwapRB);

  // Probed shapes
  std::vector<int> vision_input_shape_;
  std::vector<int> fpn_feat_0_shape_;
  std::vector<int> text_ids_shape_;

  // Image buffers (batch=1)
  tensor::Memory<float> preprocessed_images_;
  std::shared_ptr<tensor::Memory<uint8_t>> original_image_buf_;
  tensor::Memory<float> affine_matrix_;
  tensor::Memory<float> mask_affine_matrix_;

  // Vision encoder outputs. fpn_feat_2_ (the 72x72 top level) is overwritten in
  // place by apply_memory_attention() to carry the memory-conditioned embedding;
  // fpn_feat_0_/fpn_feat_1_/fpn_pos_2_ are passed to the decoder unchanged.
  tensor::Memory<float> fpn_feat_0_;
  tensor::Memory<float> fpn_feat_1_;
  tensor::Memory<float> fpn_feat_2_;
  tensor::Memory<float> fpn_pos_2_;

  // Text prompt inputs / features (batch=1)
  tensor::Memory<int64_t> text_input_ids_;
  tensor::Memory<int64_t> text_attention_mask_;
  tensor::Memory<float> text_features_;
  tensor::Memory<bool> text_mask_;

  // Decoder prompt inputs (text-only for the video path)
  tensor::Memory<float> prompt_features_;
  tensor::Memory<bool> prompt_mask_;

  // Decoder outputs (batch=1)
  tensor::Memory<float> pred_masks_;
  tensor::Memory<float> pred_boxes_;
  tensor::Memory<float> pred_logits_;
  tensor::Memory<float> presence_logits_;

  // Postprocess buffers (batch=1)
  tensor::Memory<float> filter_boxes_;
  tensor::Memory<float> filter_scores_;
  tensor::Memory<int> filter_indices_;
  tensor::Memory<int> box_count_;
  tensor::Memory<uint8_t> mask_buffer_;
  tensor::Memory<float> box_affine_matrices_;

  // --- Memory-bank buffers ---------------------------------------------------

  // Aggregate foreground mask fed to the memory encoder, at the FULL tracker
  // input resolution [K=1,1,1008,1008] (the contract's pred_mask). Built each
  // frame from the detector's per-object masks (thresholded union, upsampled).
  tensor::Memory<float> mem_pred_mask_;
  // Host staging for the aggregate mask before the H2D upload.
  tensor::Memory<float> mem_pred_mask_host_;
  // object_score_logits scalar input to the memory encoder [K=1,1].
  tensor::Memory<float> mem_obj_score_;

  // Per-frame memory-encoder outputs (one frame), SPATIAL [C=256,H=72,W=72],
  // before being rearranged to seq-major [H*W,C] and copied into a ring slot.
  tensor::Memory<float> maskmem_features_;
  tensor::Memory<float> maskmem_pos_enc_;

  // Ring of pre-allocated slots. Each slot holds one frame's maskmem feature and
  // pos buffers, both stored in SEQ-MAJOR [H*W, C] = [mem_tokens_per_frame_,
  // mem_dim_] layout so pack_memory() is a plain contiguous D2D copy into the
  // [M,1,C] memory tensor.
  struct MemorySlot {
    tensor::Memory<float> feature;
    tensor::Memory<float> pos;
  };
  std::vector<MemorySlot> mem_slots_;
  // Indices of valid slots in chronological order (front = oldest, back =
  // newest). append_memory() rotates the newest slot in and evicts the front.
  std::deque<int> mem_valid_;
  // Next slot index to (re)use as the ring rotates.
  int mem_next_slot_ = 0;

  // current_feat / current_pos for memory-attention, seq-major [H*W,1,C] =
  // [5184,1,256], derived by rearranging fpn_feat_2_ / fpn_pos_2_ from [C,H,W].
  tensor::Memory<float> current_feat_;
  tensor::Memory<float> current_pos_;

  // Host scratch for the [C,H,W] <-> [H*W,C] transpose (sized C*H*W floats,
  // holds one src and one dst plane = 2*C*H*W). See rearrange_chw_to_hwc().
  tensor::Memory<float> transpose_scratch_;

  // Contiguous packing of the valid slots consumed by memory-attention, plus the
  // bool key-padding mask. `memory`/`memory_pos` are seq-major [M,1,C] with
  // M = mem_bank_max_ * mem_tokens_per_frame_ rows max; memory_mask_ is [1,M].
  // NOTE: the RoPE encoder ignores memory_mask (no key-padding path), so we pack
  // exactly the valid slots contiguously and pass an all-true mask.
  tensor::Memory<float> mem_feat_concat_;
  tensor::Memory<float> mem_pos_concat_;
  tensor::Memory<bool> memory_mask_;

  // Memory-attention output (memory-conditioned current-frame embedding),
  // spatial [C,H,W] = [256,72,72]. Copied back into fpn_feat_2_.
  tensor::Memory<float> pix_feat_with_mem_;

  // Tokenizer
  std::unique_ptr<tokenizers::Tokenizer> tokenizer_;
};

} // namespace reusex::vision::tensor_rt

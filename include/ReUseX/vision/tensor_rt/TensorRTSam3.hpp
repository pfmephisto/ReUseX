#pragma once
#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IModel.hpp>

#include <ReUseX/vision/tensor_rt/TensorRTData.hpp>
#include <ReUseX/vision/tensor_rt/common/object.hpp>
#include <ReUseX/vision/tensor_rt/infer/sam3type.hpp>
// #include <ReUseX/vision/tensor_rt/common/device.hpp>
#include <ReUseX/vision/tensor_rt/common/memory.hpp>
#include <ReUseX/vision/tensor_rt/common/norm.hpp>
#include <ReUseX/vision/tensor_rt/common/tensorrt.hpp>
#include <unordered_map>
#include <vector>

#include <tokenizers_cpp.h>

namespace ReUseX::vision::tensor_rt {

/* TensorRTSam3 is an implementation of the SAM3 model using TensorRT for
efficient inference. It supports batch processing of images and prompts, with
careful memory management to optimize performance on GPU. The class handles the
loading of TensorRT engines for the vision encoder, text encoder, geometry
encoder, and decoder, and provides a forward method to process input data and
produce segmentation results. The implementation includes preprocessing of
images, gathering of features from the vision encoder, and post-processing of
the decoder outputs to generate final masks and bounding boxes. The design
allows for flexibility in handling different types of prompts (text and
geometry) and is optimized for use in real-time applications where latency is a
concern. */
class TensorRTSam3 : public IModel {
    private:
  using InferResult = object::DetectionBoxArray;
  using InferResultArray = std::vector<object::DetectionBoxArray>;

    public:
  /* Constructor for TensorRTSam3 without geometry encoder. Initializes the
   * model with the specified paths for the vision encoder, text encoder, and
   * decoder, and sets the GPU ID for inference.
   * @param vision_encoder_path: Path to the TensorRT engine file for the vision
   * encoder.
   * @param text_encoder_path: Path to the TensorRT engine file for the text
   * encoder.
   * @param decoder_path: Path to the TensorRT engine file for the decoder.
   * @param gpu_id: ID of the GPU to use for inference.
   */
  TensorRTSam3(const std::string vision_encoder_path,
               const std::string text_encoder_path,
               const std::string geometry_encoder_path,
               const std::string decoder_path, const std::string tokenizer_path,
               int gpu_id);

  /* Destructor for TensorRTSam3. Cleans up any resources used by the model.
   * @param model_path: Path to the model files (not used in this
   * implementation).
   * @return A unique pointer to an instance of TensorRTSam3.
   */
  static std::unique_ptr<TensorRTSam3>
  create(const std::filesystem::path &model_path);

  /* Forward method for TensorRTSam3. Takes a span of input pairs and processes
   * them through the model to produce output pairs. This method handles the
   * entire inference pipeline, including preprocessing, feature extraction,
   * decoding, and post-processing to generate the final segmentation results.
   * @param input: A span of input pairs containing the data to be processed.
   * @return A vector of output pairs containing the results of the inference.
   */
  std::vector<IDataset::Pair>
  forward(const std::span<IDataset::Pair> &input) override;

    protected:
  // virtual ~TensorRTSam3() = default;

  /* Loads the TensorRT engines for the vision encoder, text encoder, geometry
   * encoder, and decoder. This method is responsible for initializing the
   * engines based on the provided model paths and ensuring that they are ready
   * for inference. It returns true if all engines are loaded successfully, and
   * false otherwise.
   * @return A boolean indicating whether the engines were loaded successfully.
   */
  bool load_engines();

  static std::string load_bytes_from_file(const std::string &file_path);

  bool setup_geometry_input(
      const cv::Mat &image, const std::string &label,
      const std::vector<std::pair<std::string, std::array<float, 4>>> &boxes);

  /*
  // Core implementation
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    bool return_mask = false,
                                    void *stream = nullptr) override;
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    const std::string &geom_label,
                                    bool return_mask = false,
                                    void *stream = nullptr) override;
    */

    private:
  // Define internal structure for flattening Prompt
  struct PromptMeta {
    int image_idx;    // Which image this Prompt belongs to
    int original_idx; // The index of this Prompt in the original image vector
    const Sam3PromptUnit *ptr; // Pointer to the original Prompt data
  };

  // Internal processing function
  void preprocess(const TensorRTData &input, int ibatch, void *stream);

  bool encode_image(int batch_size, void *stream);

  // Modification: Gather features, collect data from Vision features according
  // to the image index corresponding to the current Prompt Batch
  void gather_vision_features(const std::vector<PromptMeta> &batch_prompts,
                              int batch_size, void *stream);

  // Modified encoding function, based on the current batch size
  bool encode_text(const std::vector<PromptMeta> &batch_prompts, int batch_size,
                   void *stream);
  bool encode_boxes(const std::vector<PromptMeta> &batch_prompts,
                    int batch_size, int max_boxes, void *stream);
  bool decode(int batch_size, int prompt_len, void *stream);

  // Post-processing
  void postprocess(InferResult &image_result, int batch_idx, int image_idx,
                   const std::string &label, float confidence_threshold,
                   bool return_mask, void *stream);

  /* Allocates memory for all the necessary buffers used during inference. This
   * method is designed to be called only once during the initialization phase
   * of the model, and it sets up the memory structures based on the maximum
   * batch sizes and input dimensions defined in the class. It ensures that all
   * buffers are properly allocated and ready for use during the forward pass,
   * optimizing memory usage and performance on the GPU.
   */
  void allocate_memory_once();

  void set_binding_dim(std::shared_ptr<TensorRT::Engine> &engine,
                       int binding_index, const std::vector<int> &dims);

    private:
  // Configuration
  bool isdynamic_model_ = true;
  int input_image_width_ = 1008;
  int input_image_height_ = 1008;
  int gpu_id_ = 0;

  // --- Batch processing limit configuration ---
  // Can be adjusted according to VRAM size
  const int max_image_batch_ =
      2; // This Vision Encoder is relatively large, limit the number of images
         // processed simultaneously
  const int max_prompt_batch_ =
      4; // Decoder is smaller, but VRAM is limited, limit the number of Prompts
         // decoded each time
  const int max_boxes_per_prompt_ =
      20; // Preset maximum number of supported Boxes

  // State variables
  std::vector<std::pair<int, int>>
      original_image_sizes_; // Size: max_image_batch_
  int num_queries_ = 200;
  int mask_height_ = 288;
  int mask_width_ = 288;

  // Model path
  std::string vision_encoder_path_;
  std::string text_encoder_path_;
  std::string geometry_encoder_path_;
  std::string decoder_path_;

  // TRT engine
  std::shared_ptr<TensorRT::Engine> vision_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> text_encoder_trt_;
  std::shared_ptr<TensorRT::Engine> decoder_trt_;
  std::shared_ptr<TensorRT::Engine> geometry_encoder_trt_;

  // TODO:
  // Data cache
  std::unordered_map<
      std::string, std::pair<std::array<int64_t, 32>, std::array<int64_t, 32>>>
      text_input_map_;

  // --- Memory management ---
  norm_image::Norm preprocess_norm_ = norm_image::Norm::alpha_beta(
      1.0f / 127.5f, -1.0f, norm_image::ChannelType::SwapRB);

  std::vector<int> vision_input_shape_;
  std::vector<int> fpn_feat_0_shape_;
  std::vector<int> text_ids_shape_;
  std::vector<int> geom_box_shape_;

  // Image Batch buffers (Size: max_image_batch_)
  tensor::Memory<float> preprocessed_images_;
  std::vector<std::shared_ptr<tensor::Memory<uint8_t>>> original_images_buf_;
  tensor::Memory<float> affine_matrix_;
  // Mask post-processing requires the corresponding matrix of the original
  // image (Size: max_image_batch_)
  tensor::Memory<float> mask_affine_matrix_;

  // Vision Encoder Outputs (Size: max_image_batch_)
  tensor::Memory<float> fpn_feat_0_;
  tensor::Memory<float> fpn_feat_1_;
  tensor::Memory<float> fpn_feat_2_;
  tensor::Memory<float> fpn_pos_2_;

  // Decoder Input Buffers (Size: max_prompt_batch_)
  // These are gathered from Vision Output
  tensor::Memory<float> fpn_feat_0_gather_;
  tensor::Memory<float> fpn_feat_1_gather_;
  tensor::Memory<float> fpn_feat_2_gather_;
  tensor::Memory<float> fpn_pos_2_gather_;

  // Prompt Inputs (Size: max_prompt_batch_)
  tensor::Memory<int64_t> text_input_ids_;
  tensor::Memory<int64_t> text_attention_mask_;

  tensor::Memory<float> geom_boxes_;
  tensor::Memory<int64_t> geom_labels_;

  tensor::Memory<float> text_features_;
  tensor::Memory<bool> text_mask_;

  tensor::Memory<float> geom_features_;
  tensor::Memory<bool> geom_mask_;

  // Used to store the results of pre-set geometry models
  std::unordered_map<std::string, std::shared_ptr<tensor::Memory<float>>>
      geom_features_cache_;
  std::unordered_map<std::string, std::shared_ptr<tensor::Memory<bool>>>
      geom_mask_cache_;

  tensor::Memory<float> prompt_features_;
  tensor::Memory<bool> prompt_mask_;

  // Decoder Output (Size: max_prompt_batch_)
  tensor::Memory<float> pred_masks_;
  tensor::Memory<float> pred_boxes_;
  tensor::Memory<float> pred_logits_;
  tensor::Memory<float> presence_logits_;

  // Postprocess (Size: max_prompt_batch_)
  tensor::Memory<float> filter_boxes_;
  tensor::Memory<float> filter_scores_;
  tensor::Memory<int> filter_indices_;
  tensor::Memory<int> box_count_;
  tensor::Memory<uint8_t> mask_buffer_;
  tensor::Memory<float>
      box_affine_matrices_; // Matrix for each Box during Mask recovery

  // Tokenizer
  std::unique_ptr<tokenizers::Tokenizer> tokenizer_;
};
} // namespace ReUseX::vision::tensor_rt

#include <ReUseX/core/logging.hpp>
#include <ReUseX/vision/tensor_rt/common/tensorrt.hpp>

#include <cuda_runtime.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <string.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvInferRuntime.h>
#include <ReUseX/vision/tensor_rt/common/check.hpp>

namespace ReUseX::vision::tensor_rt::TensorRT {

static class Logger : public nvinfer1::ILogger {
    public:
  void log(Severity severity, const char *msg) noexcept override {
    ReUseX::core::log(to_log_level(severity), "[NVINFER {}]: {}",
                to_string(severity), msg);
  }

    private:
  static ReUseX::core::LogLevel to_log_level(Severity severity) {
    switch (severity) {
    case Severity::kINTERNAL_ERROR:
    case Severity::kERROR:
      return ReUseX::core::LogLevel::error;
    case Severity::kWARNING:
      return ReUseX::core::LogLevel::warn;
    case Severity::kINFO:
      return ReUseX::core::LogLevel::info;
    case Severity::kVERBOSE:
      return ReUseX::core::LogLevel::debug;
    default:
      return ReUseX::core::LogLevel::info;
    }
  }
  static const char *to_string(Severity severity) {
    switch (severity) {
    case Severity::kINTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case Severity::kERROR:
      return "ERROR";
    case Severity::kWARNING:
      return "WARNING";
    case Severity::kINFO:
      return "INFO";
    case Severity::kVERBOSE:
      return "VERBOSE";
    default:
      return "UNKNOWN";
    }
  }

} gLogger_;

static std::string format_shape(const nvinfer1::Dims &shape) {
  return fmt::format("{}", fmt::join(shape.d, shape.d + shape.nbDims, " x "));
}

static std::vector<uint8_t> load_file(const std::string &file) {
  std::ifstream in(file, std::ios::in | std::ios::binary);
  if (!in.is_open())
    return {};

  in.seekg(0, std::ios::end);
  size_t length = in.tellg();

  std::vector<uint8_t> data;
  if (length > 0) {
    in.seekg(0, std::ios::beg);
    data.resize(length);

    in.read((char *)&data[0], length);
  }
  in.close();
  return data;
}

static const char *data_type_string(nvinfer1::DataType dt) {
  switch (dt) {
  case nvinfer1::DataType::kFLOAT:
    return "float32";
  case nvinfer1::DataType::kHALF:
    return "float16";
  case nvinfer1::DataType::kINT8:
    return "int8";
  case nvinfer1::DataType::kINT32:
    return "int32";
  case nvinfer1::DataType::kBOOL:
    return "bool";
  case nvinfer1::DataType::kUINT8:
    return "uint8";

#if NV_TENSORRT_MAJOR >= 10
  case nvinfer1::DataType::kFP8:
    return "fp8";
  case nvinfer1::DataType::kBF16:
    return "bf16";
  case nvinfer1::DataType::kINT64:
    return "int64";
  case nvinfer1::DataType::kINT4:
    return "int4";
#endif

  default:
    return "Unknow";
  }
}

template <typename _T> static void destroy_pointer(_T *ptr) {
  if (ptr)
    delete ptr;
}

class __native_engine_context {
    public:
  virtual ~__native_engine_context() { destroy(); }

  bool construct(const void *pdata, size_t size, const char *message_name) {
    destroy();

    if (pdata == nullptr || size == 0) {
      ReUseX::core::error("Construct for empty data found.");
      return false;
    }

    if (!initLibNvInferPlugins(&gLogger_, "")) {
      ReUseX::core::error("Failed to initialize TensorRT's plugin library.");
      return false;
    }

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger_),
        destroy_pointer<nvinfer1::IRuntime>);
    if (runtime_ == nullptr) {
      ReUseX::core::error("Failed to create tensorRT runtime: {}.", message_name);
      return false;
    }

    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(pdata, size),
        destroy_pointer<nvinfer1::ICudaEngine>);

    if (engine_ == nullptr) {
      ReUseX::core::error("Failed to deserialize engine: {}.", message_name);
      return false;
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext(),
        destroy_pointer<nvinfer1::IExecutionContext>);
    if (context_ == nullptr) {
      ReUseX::core::error("Failed to create execution context: {}.", message_name);
      return false;
    }
    return context_ != nullptr;
  }

    private:
  void destroy() {
    context_.reset();
    engine_.reset();
    runtime_.reset();
  }

    public:
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IRuntime> runtime_ = nullptr;
};

class EngineImplement : public Engine {
    public:
  std::shared_ptr<__native_engine_context> context_;
  std::unordered_map<std::string, int> binding_name_to_index_;

  virtual ~EngineImplement() = default;

  bool construct(const void *data, size_t size, const char *message_name) {
    context_ = std::make_shared<__native_engine_context>();
    if (!context_->construct(data, size, message_name)) {
      return false;
    }

    setup();
    return true;
  }

  bool load(const std::string &file) {
    auto data = load_file(file);
    if (data.empty()) {
      ReUseX::core::error(
          "An empty file has been loaded. Please confirm your file path: {}.",
          file);
      return false;
    }
    return this->construct(data.data(), data.size(), file.c_str());
  }

  void setup() {
    auto engine = this->context_->engine_;
    int nbBindings = engine->getNbIOTensors();

    binding_name_to_index_.clear();
    for (int i = 0; i < nbBindings; ++i) {
      const char *bindingName = engine->getIOTensorName(i);
      binding_name_to_index_[bindingName] = i;
    }
  }

  virtual int index(const std::string &name) override {
    auto iter = binding_name_to_index_.find(name);
    Assertf(iter != binding_name_to_index_.end(),
            "Can not found the binding name: %s", name.c_str());
    return iter->second;
  }

  virtual bool
  forward(const std::unordered_map<std::string, const void *> &bindings,
          void *stream, void *input_consum_event) override {
    auto engine = this->context_->engine_;
    auto context = this->context_->context_;
    int ibinding = 0;
    for (; ibinding < engine->getNbIOTensors(); ++ibinding) {
      auto tensor_name = engine->getIOTensorName(ibinding);
      auto binding_iter = bindings.find(tensor_name);
      if (binding_iter == bindings.end()) {
        ReUseX::core::error("Failed to set the tensor address, can not found tensor "
                      "{} in bindings provided.",
                      tensor_name);
        return false;
      }

      if (!context->setTensorAddress(tensor_name,
                                     (void *)binding_iter->second)) {
        ReUseX::core::error("Failed to set tensor address for tensor {}.",
                      tensor_name);
        return false;
      }
    }
    return context->enqueueV3((cudaStream_t)stream);
  }

  virtual std::vector<int> run_dims(const std::string &name) override {
    return run_dims(index(name));
  }

  virtual std::vector<int> run_dims(int ibinding) override {
    auto engine = this->context_->engine_;
    auto context = this->context_->context_;
    auto dim = context->getTensorShape(engine->getIOTensorName(ibinding));
    return std::vector<int>(dim.d, dim.d + dim.nbDims);
  }

  virtual std::vector<int> static_dims(const std::string &name) override {
    return static_dims(index(name));
  }

  virtual std::vector<int> static_dims(int ibinding) override {
    auto engine = this->context_->engine_;
    auto dim = engine->getTensorShape(engine->getIOTensorName(ibinding));
    return std::vector<int>(dim.d, dim.d + dim.nbDims);
  }

  virtual int num_bindings() override {
    return this->context_->engine_->getNbIOTensors();
  }

  virtual bool is_input(int ibinding) override {
    auto engine = this->context_->engine_;
    return engine->getTensorIOMode(engine->getIOTensorName(ibinding)) ==
           nvinfer1::TensorIOMode::kINPUT;
  }

  virtual bool is_input(const std::string &name) override {
    auto engine = this->context_->engine_;
    return engine->getTensorIOMode(name.c_str()) ==
           nvinfer1::TensorIOMode::kINPUT;
  }

  virtual bool set_run_dims(const std::string &name,
                            const std::vector<int> &dims) override {
    return this->set_run_dims(index(name), dims);
  }

  virtual bool set_run_dims(int ibinding,
                            const std::vector<int> &dims) override {
    nvinfer1::Dims d;
    for (int i = 0; i < dims.size(); ++i) {
      d.d[i] = static_cast<int64_t>(dims[i]); // Convert to int64_t
    }
    // memcpy(d.d, dims.data(), sizeof(int) * dims.size());
    d.nbDims = dims.size();
    auto engine = this->context_->engine_;
    auto context = this->context_->context_;
    // return context->setInputShape("images", nvinfer1::Dims{4, {1, 3, 640,
    // 640}});
    return context->setInputShape(engine->getIOTensorName(ibinding), d);
  }

  virtual int numel(const std::string &name) override {
    return numel(index(name));
  }

  virtual int numel(int ibinding) override {
    auto dim = this->run_dims(ibinding);
    return std::accumulate(dim.begin(), dim.end(), 1, std::multiplies<int>());
  }

  virtual DType dtype(const std::string &name) override {
    return dtype(index(name));
  }

  virtual DType dtype(int ibinding) override {
    auto engine = this->context_->engine_;
    auto dtype = engine->getTensorDataType(engine->getIOTensorName(ibinding));
    switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return DType::FLOAT;
    case nvinfer1::DataType::kHALF:
      return DType::HALF;
    case nvinfer1::DataType::kINT8:
      return DType::INT8;
    case nvinfer1::DataType::kINT32:
      return DType::INT32;
    case nvinfer1::DataType::kBOOL:
      return DType::BOOL;
    case nvinfer1::DataType::kUINT8:
      return DType::UINT8;

#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kFP8:
      return DType::FP8;
    case nvinfer1::DataType::kBF16:
      return DType::BF16;
    case nvinfer1::DataType::kINT64:
      return DType::INT64;
    case nvinfer1::DataType::kINT4:
      return DType::INT4;
#endif

    default:
      return DType::NONE;
    }
  }

  virtual bool has_dynamic_dim() override {
    // check if any input or output bindings have dynamic shapes
    // code from ChatGPT
    int numBindings = this->num_bindings();
    for (int i = 0; i < numBindings; ++i) {
      auto dims = this->static_dims(i);
      for (size_t j = 0; j < dims.size(); ++j) {
        if (dims[j] == -1)
          return true;
      }
    }
    return false;
  }

  virtual void print(const char *name) override {
    ReUseX::core::info("-------------------------------------------------------");

    ReUseX::core::info("{} 🌱 is {} model", name,
                 has_dynamic_dim() ? "Dynamic Shape" : "Static Shape");

    int num_input = 0;
    int num_output = 0;
    auto engine = this->context_->engine_;
    for (int i = 0; i < this->num_bindings(); ++i) {
      if (this->is_input(i))
        num_input++;
      else
        num_output++;
    }

    ReUseX::core::info("Inputs: {}", num_input);
    for (int i = 0; i < num_input; ++i) {
      auto name = engine->getIOTensorName(i);
      auto dim = engine->getTensorShape(name);
      auto dtype = engine->getTensorDataType(name);
      ReUseX::core::info("\t{}.{} : {{{}}} [{}]", i, name, format_shape(dim),
                   data_type_string(dtype));
    }

    ReUseX::core::info("Outputs: {}", num_output);
    for (int i = 0; i < num_output; ++i) {
      auto name = engine->getIOTensorName(i + num_input);
      auto dim = engine->getTensorShape(name);
      auto dtype = engine->getTensorDataType(name);
      ReUseX::core::info("\t{}.{} : {{{}}} [{}]", i, name, format_shape(dim),
                   data_type_string(dtype));
    }
    ReUseX::core::info("-------------------------------------------------------");
  }
};

std::shared_ptr<Engine> load(const std::string &file) {
  std::shared_ptr<EngineImplement> impl(new EngineImplement());
  if (!impl->load(file))
    impl.reset();
  return impl;
}
}; // namespace ReUseX::vision::tensor_rt::TensorRT

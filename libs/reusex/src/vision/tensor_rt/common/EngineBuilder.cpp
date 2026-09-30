// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/tensor_rt/common/EngineBuilder.hpp"
#include "core/logging.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>

#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace reusex::vision::tensor_rt {

namespace {

/// Bridge TensorRT's builder logging into the ReUseX logging facade. Kept local
/// to the builder TU (the runtime path in tensorrt.cpp has its own copy).
class BuilderLogger : public nvinfer1::ILogger {
    public:
  void log(Severity severity, const char *msg) noexcept override {
    reusex::core::LogLevel level;
    switch (severity) {
    case Severity::kINTERNAL_ERROR:
    case Severity::kERROR:
      level = reusex::core::LogLevel::error;
      break;
    case Severity::kWARNING:
      level = reusex::core::LogLevel::warn;
      break;
    case Severity::kINFO:
      level = reusex::core::LogLevel::debug; // builder INFO is very chatty
      break;
    default:
      level = reusex::core::LogLevel::trace;
      break;
    }
    reusex::core::log(level, "[NVINFER-build] {}", msg);
  }
};

BuilderLogger &builder_logger() {
  static BuilderLogger logger;
  return logger;
}

// TensorRT 10 objects are released with `delete` (public virtual destructors),
// so a default_delete unique_ptr is the idiomatic owner.
template <typename T> using TrtPtr = std::unique_ptr<T>;

nvinfer1::Dims to_dims(const std::vector<int> &v, const std::string &engine,
                       const std::string &input) {
  nvinfer1::Dims d;
  if (v.empty() || v.size() > nvinfer1::Dims::MAX_DIMS)
    throw std::runtime_error(
        fmt::format("EngineBuilder: {}: input '{}' has an invalid rank {}",
                    engine, input, v.size()));
  d.nbDims = static_cast<int32_t>(v.size());
  for (size_t i = 0; i < v.size(); ++i)
    d.d[i] = static_cast<int64_t>(v[i]);
  return d;
}

void write_atomic(const std::filesystem::path &path, const void *data,
                  size_t size) {
  std::filesystem::create_directories(path.parent_path());
  const std::filesystem::path tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out.is_open())
      throw std::runtime_error(fmt::format(
          "EngineBuilder: cannot open {} for writing", tmp.string()));
    out.write(static_cast<const char *>(data),
              static_cast<std::streamsize>(size));
    if (!out.good())
      throw std::runtime_error(
          fmt::format("EngineBuilder: failed writing {}", tmp.string()));
  }
  std::filesystem::rename(tmp, path);
}

} // namespace

std::filesystem::path build_engine(const EngineBuildRequest &req) {
  const std::string name = req.onnx_path.stem().string();

  if (!std::filesystem::exists(req.onnx_path))
    throw std::runtime_error(fmt::format("EngineBuilder: ONNX not found: {}",
                                         req.onnx_path.string()));

  reusex::info("EngineBuilder: building '{}' ({}) → {}", name,
               req.profile.precision, req.engine_path.string());

  // SAM3's exported graphs use only standard ops after the export fixes, but
  // initialise the plugin registry anyway so a plugin-backed op (if introduced)
  // resolves during parse.
  initLibNvInferPlugins(&builder_logger(), "");

  TrtPtr<nvinfer1::IBuilder> builder(
      nvinfer1::createInferBuilder(builder_logger()));
  if (!builder)
    throw std::runtime_error("EngineBuilder: createInferBuilder failed");

  // TensorRT 10 networks are always explicit-batch; no creation flags needed.
  TrtPtr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
  if (!network)
    throw std::runtime_error("EngineBuilder: createNetworkV2 failed");

  TrtPtr<nvonnxparser::IParser> parser(
      nvonnxparser::createParser(*network, builder_logger()));
  if (!parser)
    throw std::runtime_error("EngineBuilder: createParser failed");

  if (!parser->parseFromFile(
          req.onnx_path.string().c_str(),
          static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    std::string errs;
    for (int i = 0; i < parser->getNbErrors(); ++i)
      errs += fmt::format("\n  [{}] {}", i, parser->getError(i)->desc());
    throw std::runtime_error(fmt::format(
        "EngineBuilder: failed to parse ONNX {}:{}", req.onnx_path.string(),
        errs.empty() ? " (no detail)" : errs));
  }

  TrtPtr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
  if (!config)
    throw std::runtime_error("EngineBuilder: createBuilderConfig failed");

  const std::size_t workspace_bytes =
      static_cast<std::size_t>(req.profile.workspace_mb) << 20;
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                             workspace_bytes);

  // Precision. fp32 is the default; the bf16-native vision-encoder MUST stay
  // fp32 (fp16 corrupts it to all-background), which engine-build.json encodes.
  if (req.profile.is_fp16()) {
    if (builder->platformHasFastFp16())
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    else
      reusex::warn("EngineBuilder: {}: platform lacks fast fp16; building fp32",
                   name);
  }

  // Optimization profile (dynamic and static-but-specified inputs).
  if (!req.profile.shapes.empty()) {
    nvinfer1::IOptimizationProfile *opt = builder->createOptimizationProfile();
    for (const auto &[input, tri] : req.profile.shapes) {
      opt->setDimensions(input.c_str(), nvinfer1::OptProfileSelector::kMIN,
                         to_dims(tri.min, name, input));
      opt->setDimensions(input.c_str(), nvinfer1::OptProfileSelector::kOPT,
                         to_dims(tri.opt, name, input));
      opt->setDimensions(input.c_str(), nvinfer1::OptProfileSelector::kMAX,
                         to_dims(tri.max, name, input));
    }
    config->addOptimizationProfile(opt);
  }

  reusex::info("EngineBuilder: {}: parsing done, invoking builder "
               "(workspace {} MiB, {}) — this can take minutes",
               name, req.profile.workspace_mb, req.profile.precision);

  TrtPtr<nvinfer1::IHostMemory> serialized(
      builder->buildSerializedNetwork(*network, *config));
  if (!serialized || serialized->size() == 0)
    throw std::runtime_error(fmt::format(
        "EngineBuilder: buildSerializedNetwork returned empty for '{}' "
        "(out of memory, or an unsupported layer under the requested "
        "precision)",
        name));

  write_atomic(req.engine_path, serialized->data(), serialized->size());
  reusex::info("EngineBuilder: wrote {} ({:.1f} MiB)", req.engine_path.string(),
               serialized->size() / (1024.0 * 1024.0));
  return req.engine_path;
}

} // namespace reusex::vision::tensor_rt

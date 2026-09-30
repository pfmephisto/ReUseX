// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/vision/sam3/EngineBuildProfiles.hpp"

#include <filesystem>

namespace reusex::vision::tensor_rt {

/// One ONNX → TensorRT engine build job.
struct EngineBuildRequest {
  /// Input ONNX graph (external weight files, if any, must sit beside it).
  std::filesystem::path onnx_path;
  /// Output serialized engine (written atomically via a temp file + rename).
  std::filesystem::path engine_path;
  /// Precision, workspace and optimization-profile shapes for this engine,
  /// taken verbatim from ``engine-build.json`` (already fp32-rule-resolved).
  sam3::EngineProfile profile;
};

/// Build a serialized TensorRT engine from an ONNX file using nvonnxparser +
/// IBuilder, applying ``req.profile`` (precision, workspace, optimization
/// profile). This is the native, self-contained equivalent of the Python
/// ``trtexec`` driver — it lets the C++ side build the GPU-specific engines on
/// first use instead of shipping prebuilt (non-portable) ``.engine`` files.
///
/// The parent directory of ``engine_path`` is created if needed. Throws
/// ``std::runtime_error`` on any parse/build/IO failure. Returns the engine
/// path on success.
std::filesystem::path build_engine(const EngineBuildRequest &req);

} // namespace reusex::vision::tensor_rt

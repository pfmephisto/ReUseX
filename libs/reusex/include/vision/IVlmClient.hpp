// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Provider-agnostic interface for a vision-language model that describes an
// image crop (#373). The concrete OpenAiCompatibleVlmClient talks to any
// OpenAI-compatible chat/completions endpoint (local Ollama by default, or a
// cloud provider). The interface exists so the `describe()` stage can be driven
// by a fake client in tests without any network.

#include "reusex/vision/vlm_protocol.hpp"

#include <string>

// Forward-declared to keep OpenCV out of this public header (STANDARDS §2).
namespace cv {
class Mat;
}

namespace reusex::vision {

/// A model that, given an image crop and a text prompt, returns structured
/// attributes. Implementations must not throw on an empty/degenerate model
/// answer — they return a VlmResult with `ok == false` so the caller can log
/// and skip rather than fabricate (STANDARDS §5). Transport failures
/// (connection refused, HTTP error) may throw.
struct IVlmClient {
  virtual ~IVlmClient() = default;

  /// Describe @p crop (a BGR CV_8UC3 image) given @p prompt.
  virtual VlmResult describe(const cv::Mat &crop,
                             const std::string &prompt) = 0;
};

} // namespace reusex::vision

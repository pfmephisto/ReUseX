// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Concrete IVlmClient talking to any OpenAI-compatible /chat/completions
// endpoint over libcurl (#373). Module-private: constructed by describe().

#include "reusex/vision/IVlmClient.hpp"

#include <string>

namespace reusex::vision {

class OpenAiCompatibleVlmClient : public IVlmClient {
    public:
  /// @param base_url  e.g. "http://localhost:11434/v1" (the "/chat/completions"
  ///                  suffix is appended if not already present).
  /// @param model     Model name.
  /// @param api_key   Optional bearer token; empty disables the header.
  OpenAiCompatibleVlmClient(std::string base_url, std::string model,
                            std::string api_key);

  VlmResult describe(const cv::Mat &crop, const std::string &prompt) override;

    private:
  std::string endpoint_; ///< Fully-resolved chat/completions URL.
  std::string model_;
  std::string api_key_;
};

} // namespace reusex::vision

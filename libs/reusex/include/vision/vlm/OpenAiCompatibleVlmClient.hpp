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
  /// @param base_url          e.g. "http://localhost:11434/v1" (the
  ///                          "/chat/completions" suffix is appended if
  ///                          absent).
  /// @param model             Model name.
  /// @param api_key           Optional bearer token; empty disables the header.
  /// @param connect_timeout_s CURLOPT_CONNECTTIMEOUT value in seconds (0 = no
  ///                          limit). Required for multithreaded programs to
  ///                          avoid blocking forever on a hung endpoint.
  /// @param total_timeout_s   CURLOPT_TIMEOUT value in seconds (0 = no limit).
  OpenAiCompatibleVlmClient(std::string base_url, std::string model,
                            std::string api_key, int connect_timeout_s = 10,
                            int total_timeout_s = 120);

  VlmResult describe(const cv::Mat &crop, const std::string &prompt) override;

  int connect_timeout_s() const { return connect_timeout_s_; }
  int total_timeout_s() const { return total_timeout_s_; }

    private:
  std::string endpoint_; ///< Fully-resolved chat/completions URL.
  std::string model_;
  std::string api_key_;
  int connect_timeout_s_;
  int total_timeout_s_;
};

} // namespace reusex::vision

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/vlm/OpenAiCompatibleVlmClient.hpp"

#include "core/logging.hpp"
#include "reusex/vision/vlm_protocol.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::vision {

namespace {

// Standard base64 encoder (RFC 4648). Kept local: vision links no base64 lib,
// and the transport is not otherwise dependency-heavy.
std::string base64_encode(const std::vector<uint8_t> &data) {
  static constexpr char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);
  size_t i = 0;
  for (; i + 2 < data.size(); i += 3) {
    const uint32_t n = (uint32_t(data[i]) << 16) |
                       (uint32_t(data[i + 1]) << 8) | uint32_t(data[i + 2]);
    out.push_back(tbl[(n >> 18) & 0x3F]);
    out.push_back(tbl[(n >> 12) & 0x3F]);
    out.push_back(tbl[(n >> 6) & 0x3F]);
    out.push_back(tbl[n & 0x3F]);
  }
  if (i < data.size()) {
    uint32_t n = uint32_t(data[i]) << 16;
    if (i + 1 < data.size())
      n |= uint32_t(data[i + 1]) << 8;
    out.push_back(tbl[(n >> 18) & 0x3F]);
    out.push_back(tbl[(n >> 12) & 0x3F]);
    out.push_back(i + 1 < data.size() ? tbl[(n >> 6) & 0x3F] : '=');
    out.push_back('=');
  }
  return out;
}

size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *response = static_cast<std::string *>(userdata);
  response->append(ptr, size * nmemb);
  return size * nmemb;
}

std::string resolve_endpoint(std::string base_url) {
  while (!base_url.empty() && base_url.back() == '/')
    base_url.pop_back();
  if (base_url.size() >= std::string("/chat/completions").size() &&
      base_url.rfind("/chat/completions") ==
          base_url.size() - std::string("/chat/completions").size())
    return base_url;
  return base_url + "/chat/completions";
}

} // namespace

OpenAiCompatibleVlmClient::OpenAiCompatibleVlmClient(std::string base_url,
                                                     std::string model,
                                                     std::string api_key,
                                                     int connect_timeout_s,
                                                     int total_timeout_s)
    : endpoint_(resolve_endpoint(std::move(base_url))),
      model_(std::move(model)), api_key_(std::move(api_key)),
      connect_timeout_s_(connect_timeout_s), total_timeout_s_(total_timeout_s) {
}

VlmResult OpenAiCompatibleVlmClient::describe(const cv::Mat &crop,
                                              const std::string &prompt) {
  // Encode the crop as JPEG, then base64.
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", crop, jpeg) || jpeg.empty())
    throw std::runtime_error("VLM client: cv::imencode(.jpg) failed on crop");
  const std::string b64 = base64_encode(jpeg);

  const std::string body = build_chat_request_json(model_, b64, prompt).dump();

  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("VLM client: curl_easy_init failed");

  std::string response;
  struct curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  if (!api_key_.empty())
    headers = curl_slist_append(headers,
                                ("Authorization: Bearer " + api_key_).c_str());

  // Required for CURLOPT_TIMEOUT to work reliably in multithreaded programs
  // (otherwise libcurl uses SIGALRM, which is process-wide).
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  if (connect_timeout_s_ > 0)
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT,
                     static_cast<long>(connect_timeout_s_));
  if (total_timeout_s_ > 0)
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,
                     static_cast<long>(total_timeout_s_));
  curl_easy_setopt(curl, CURLOPT_URL, endpoint_.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK)
    throw std::runtime_error(std::string("VLM request failed: ") +
                             curl_easy_strerror(res));
  if (http_code < 200 || http_code >= 300)
    throw std::runtime_error("VLM request failed (HTTP " +
                             std::to_string(http_code) + "): " + response);

  return parse_vlm_response(response);
}

} // namespace reusex::vision

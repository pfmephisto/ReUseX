// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Provider-agnostic OpenAI-compatible chat/completions protocol for the VLM
// attributes pass (#373).
//
// Deliberately header-only and network-free: the two functions here — building
// the request body and parsing the response body — are pure string/JSON
// transforms with NO dependency on libcurl or reusex_vision. That lets them be
// unit-tested directly in the LIGHT test binary (which does not link
// reusex_vision), which is the whole point: the transport is a thin wrapper
// nobody tests in CI, but the request shape and the (defensive) parser are
// where the bugs live, so those must be testable without a server.

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex::vision {

/// The description + arbitrary attributes a vision-language model returned for
/// one crop. The prompt fully drives which keys come back, so there is no fixed
/// schema: a free-text @c description plus a list of key/value pairs. The
/// parser never fabricates a value (STANDARDS §5).
struct VlmResult {
  std::string description; ///< Free-text description of the object/material.
  std::vector<std::pair<std::string, std::string>>
      attributes;       ///< Arbitrary key/value pairs the model returned.
  std::string raw_json; ///< The raw response body, verbatim.
  bool ok = false;      ///< True only when a non-empty description OR at least
                        ///< one attribute was recovered (never fabricated).
};

/// Build an OpenAI-compatible chat/completions request body for one image.
///
/// @param model    Model name the endpoint should route to (e.g. "qwen2.5vl").
/// @param b64_jpeg Base64-encoded JPEG bytes of the object crop (no data URI
///                 prefix — this function adds the `data:image/jpeg;base64,`).
/// @param prompt   The instruction text sent alongside the image.
///
/// Determinism: temperature is pinned to 0. JSON output is requested via BOTH
/// the Ollama `format: "json"` key and the OpenAI `response_format:
/// {type:"json_object"}` key — a server ignores whichever it does not know, so
/// including both keeps the same body usable across providers (STANDARDS §6).
inline nlohmann::json build_chat_request_json(const std::string &model,
                                              const std::string &b64_jpeg,
                                              const std::string &prompt) {
  nlohmann::json image_url;
  image_url["url"] = "data:image/jpeg;base64," + b64_jpeg;

  nlohmann::json content = nlohmann::json::array();
  content.push_back({{"type", "text"}, {"text", prompt}});
  content.push_back({{"type", "image_url"}, {"image_url", image_url}});

  nlohmann::json message;
  message["role"] = "user";
  message["content"] = content;

  nlohmann::json req;
  req["model"] = model;
  req["messages"] = nlohmann::json::array({message});
  req["temperature"] = 0;
  req["stream"] = false;
  // Ollama's native key.
  req["format"] = "json";
  // OpenAI's key (ignored by servers that don't support it).
  req["response_format"] = {{"type", "json_object"}};
  return req;
}

namespace detail {

/// Strip a leading/trailing markdown ```json ... ``` fence if present, and trim
/// surrounding whitespace. Returns the inner text (which may still not be
/// JSON).
inline std::string strip_json_fence(std::string s) {
  auto trim = [](std::string &t) {
    const char *ws = " \t\r\n";
    t.erase(0, t.find_first_not_of(ws));
    const auto end = t.find_last_not_of(ws);
    t.erase(end == std::string::npos ? 0 : end + 1);
  };
  trim(s);
  if (s.rfind("```", 0) == 0) {
    // Drop the opening fence line (```json or ```), then the closing fence.
    const auto nl = s.find('\n');
    if (nl != std::string::npos)
      s.erase(0, nl + 1);
    else
      s.erase(0, 3);
    const auto close = s.rfind("```");
    if (close != std::string::npos)
      s.erase(close);
    trim(s);
  }
  return s;
}

/// Whether a parsed body is an OpenAI-/Ollama-style chat envelope, i.e. it
/// carries an assistant message under `choices[0].message.content` or
/// `message.content` (regardless of whether that content is empty).
inline bool is_chat_envelope(const nlohmann::json &body) {
  if (!body.is_object())
    return false;
  if (body.contains("choices") && body["choices"].is_array() &&
      !body["choices"].empty()) {
    const auto &first = body["choices"][0];
    if (first.is_object() && first.contains("message") &&
        first["message"].is_object() && first["message"].contains("content"))
      return true;
  }
  // Ollama's /api/chat shape: { "message": { "content": "..." } }
  if (body.contains("message") && body["message"].is_object() &&
      body["message"].contains("content"))
    return true;
  return false;
}

/// Pull the assistant message text out of an OpenAI-style chat response.
/// Returns an empty string when no string content is present; callers must
/// distinguish "no envelope" from "envelope with empty content" via
/// is_chat_envelope() rather than treating an empty return as "fall back to the
/// whole body" (an empty assistant reply means the model returned nothing
/// usable, STANDARDS §5).
inline std::string extract_message_content(const nlohmann::json &body) {
  if (body.is_object() && body.contains("choices") &&
      body["choices"].is_array() && !body["choices"].empty()) {
    const auto &first = body["choices"][0];
    if (first.contains("message") && first["message"].contains("content") &&
        first["message"]["content"].is_string())
      return first["message"]["content"].get<std::string>();
  }
  // Ollama's /api/chat shape: { "message": { "content": "..." } }
  if (body.is_object() && body.contains("message") &&
      body["message"].contains("content") &&
      body["message"]["content"].is_string())
    return body["message"]["content"].get<std::string>();
  return {};
}

/// Stringify a JSON scalar (string / number / bool). Returns nullopt for a
/// container (object/array) or null — those are not attribute values.
inline std::optional<std::string> scalar_to_string(const nlohmann::json &v) {
  if (v.is_string())
    return v.get<std::string>();
  if (v.is_boolean())
    return v.get<bool>() ? std::string("true") : std::string("false");
  if (v.is_number_integer())
    return std::to_string(v.get<long long>());
  if (v.is_number_unsigned())
    return std::to_string(v.get<unsigned long long>());
  if (v.is_number_float())
    return v.dump(); // shortest round-trippable form
  return std::nullopt;
}

/// True for any of the keys treated as the free-text description slot.
inline bool is_description_key(const std::string &k) {
  return k == "description" || k == "desc" || k == "summary";
}

} // namespace detail

/// Parse an OpenAI-compatible chat/completions response into a VlmResult.
///
/// Defensive by design (STANDARDS §5): the assistant's content is often JSON
/// wrapped in a markdown ```json fence, so this function unwraps the fence and
/// parses whatever JSON object it finds. It accepts BOTH shapes the prompt can
/// elicit:
///   - nested: `{ "description": "...", "attributes": { k: v, ... } }` — the
///     description is the free text and every scalar under `attributes` is a
///     key/value pair;
///   - flat: any object where a `description`/`desc`/`summary` key is the free
///     text and every OTHER scalar field is treated as an attribute.
/// Numbers and bools become stringified attribute values; nested containers are
/// ignored (they are not scalar values). Nothing is fabricated.
///
/// `ok` is true ONLY when a non-empty description OR at least one attribute was
/// recovered. In particular:
///   - a chat envelope whose assistant content is empty does NOT fall back to
///     re-parsing the whole envelope object (that would "recover" the envelope
///     itself and falsely report success on a model that returned nothing);
///   - an object carrying neither a description nor any attribute leaves `ok`
///     false so the stage never stores an all-empty annotation.
/// The full response body is always preserved in `raw_json`. Attributes are
/// returned in the model's key order de-duplicated (last value wins), which the
/// storage layer then keys deterministically.
inline VlmResult parse_vlm_response(const std::string &body) {
  VlmResult result;
  result.raw_json = body;

  nlohmann::json envelope = nlohmann::json::parse(body, nullptr, false);

  std::string content;
  if (!envelope.is_discarded() && detail::is_chat_envelope(envelope)) {
    // A chat envelope was returned: the payload is its assistant content, and
    // ONLY that. If the content is empty the model returned nothing usable —
    // do not fall back to re-parsing the envelope object itself.
    content = detail::extract_message_content(envelope);
    if (content.empty())
      return result; // ok stays false
  } else {
    // No chat envelope (or the body did not parse): treat the whole body as the
    // candidate payload, so a bare attributes JSON object still parses.
    content = body;
  }

  const std::string inner = detail::strip_json_fence(content);
  nlohmann::json obj = nlohmann::json::parse(inner, nullptr, false);
  if (obj.is_discarded() || !obj.is_object())
    return result; // ok stays false

  // Collect attributes de-duplicated (last value wins), preserving first-seen
  // key order so a caller sees a stable list even before the DB re-sorts it.
  std::vector<std::pair<std::string, std::string>> attrs;
  auto put_attr = [&attrs](const std::string &key, const std::string &value) {
    for (auto &kv : attrs) {
      if (kv.first == key) {
        kv.second = value;
        return;
      }
    }
    attrs.emplace_back(key, value);
  };

  // Description: a description/desc/summary key at the top level.
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    if (detail::is_description_key(it.key()) && it.value().is_string()) {
      result.description = it.value().get<std::string>();
      break;
    }
  }

  // Nested attributes object, if present.
  if (obj.contains("attributes") && obj["attributes"].is_object()) {
    for (auto it = obj["attributes"].begin(); it != obj["attributes"].end();
         ++it) {
      if (auto v = detail::scalar_to_string(it.value()))
        put_attr(it.key(), *v);
    }
  }

  // Flat attributes: every OTHER top-level scalar field. Skips the description
  // key(s) and the nested `attributes` object handled above.
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    if (detail::is_description_key(it.key()))
      continue;
    if (it.key() == "attributes" && it.value().is_object())
      continue;
    if (auto v = detail::scalar_to_string(it.value()))
      put_attr(it.key(), *v);
  }

  result.attributes = std::move(attrs);

  // Success requires a non-empty description OR at least one attribute: never
  // report ok on an object that carried neither (STANDARDS §5, no empty row).
  result.ok = !result.description.empty() || !result.attributes.empty();
  return result;
}

} // namespace reusex::vision

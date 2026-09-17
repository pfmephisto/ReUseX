// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Network-free tests for the OpenAI-compatible VLM protocol helpers (#373).
// These live in the LIGHT test binary: vlm_protocol.hpp is header-only and
// depends only on nlohmann_json, so no reusex_vision link is needed.

#include <catch2/catch_test_macros.hpp>

#include <vision/vlm_protocol.hpp>

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

using reusex::vision::build_chat_request_json;
using reusex::vision::parse_vlm_response;
using reusex::vision::VlmResult;

namespace {

// Look up an attribute value by key in the result's kv list.
std::optional<std::string> attr(const VlmResult &r, const std::string &key) {
  for (const auto &[k, v] : r.attributes)
    if (k == key)
      return v;
  return std::nullopt;
}

} // namespace

TEST_CASE("BuildChatRequestJson_ImageAndPrompt_ProducesOpenAiShape",
          "[vision][vlm][protocol]") {
  auto req = build_chat_request_json("qwen2.5vl", "QUJD", "describe this");

  CHECK(req["model"] == "qwen2.5vl");
  CHECK(req["temperature"] == 0);
  // Both determinism/JSON keys present so any provider is satisfied.
  CHECK(req["format"] == "json");
  CHECK(req["response_format"]["type"] == "json_object");

  const auto &content = req["messages"][0]["content"];
  REQUIRE(content.is_array());
  REQUIRE(content.size() == 2);
  CHECK(content[0]["type"] == "text");
  CHECK(content[0]["text"] == "describe this");
  CHECK(content[1]["type"] == "image_url");
  CHECK(content[1]["image_url"]["url"] == "data:image/jpeg;base64,QUJD");
}

TEST_CASE("ParseVlmResponse_NestedDescriptionAndAttributes_ParsesBoth",
          "[vision][vlm][protocol]") {
  const std::string body = R"({
    "choices": [
      {"message": {"content":
        "{\"description\":\"a wooden chair\",\"attributes\":{\"material\":\"wood\",\"condition\":\"good\",\"legs\":4}}"}}
    ]
  })";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a wooden chair");
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "wood");
  CHECK(*attr(r, "condition") == "good");
  // A number becomes a stringified attribute value.
  REQUIRE(attr(r, "legs").has_value());
  CHECK(*attr(r, "legs") == "4");
  CHECK_FALSE(r.raw_json.empty());
}

TEST_CASE("ParseVlmResponse_FlatObject_DescriptionKeyAndOtherFieldsAsKv",
          "[vision][vlm][protocol]") {
  const std::string body = R"({
    "choices": [
      {"message": {"content":
        "{\"description\":\"a glass pane\",\"material\":\"glass\",\"transparent\":true}"}}
    ]
  })";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a glass pane");
  // "material" is a flat field -> an attribute; "description" is NOT.
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "glass");
  // A bool becomes "true"/"false".
  REQUIRE(attr(r, "transparent").has_value());
  CHECK(*attr(r, "transparent") == "true");
  CHECK_FALSE(attr(r, "description").has_value());
}

TEST_CASE("ParseVlmResponse_MarkdownFencedContent_StripsFence",
          "[vision][vlm][protocol]") {
  const std::string body = R"({
    "choices": [
      {"message": {"content": "```json\n{\"description\":\"a window\",\"material\":\"glass\"}\n```"}}
    ]
  })";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a window");
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "glass");
}

TEST_CASE("ParseVlmResponse_OllamaMessageShape_ExtractsContent",
          "[vision][vlm][protocol]") {
  const std::string body =
      R"({"message":{"content":"{\"description\":\"a metal table\",\"material\":\"metal\"}"}})";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a metal table");
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "metal");
}

TEST_CASE("ParseVlmResponse_AltDescriptionKey_Summary",
          "[vision][vlm][protocol]") {
  // "summary" is an accepted alias for the description slot.
  const std::string body = R"({"summary":"a door","material":"wood"})";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a door");
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "wood");
}

TEST_CASE("ParseVlmResponse_AttributesOnly_NoDescription_ReturnsOk",
          "[vision][vlm][protocol]") {
  const std::string body = R"({"material":"steel","condition":"rusty"})";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok); // at least one attribute recovered
  CHECK(r.description.empty());
  CHECK(*attr(r, "material") == "steel");
  CHECK(*attr(r, "condition") == "rusty");
}

TEST_CASE("ParseVlmResponse_DescriptionOnly_NoAttributes_ReturnsOk",
          "[vision][vlm][protocol]") {
  const std::string body = R"({"description":"a plain surface"})";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "a plain surface");
  CHECK(r.attributes.empty());
}

TEST_CASE("ParseVlmResponse_NonJsonContent_ReturnsNotOk",
          "[vision][vlm][protocol]") {
  const std::string body =
      R"({"choices":[{"message":{"content":"I cannot tell what this is."}}]})";
  auto r = parse_vlm_response(body);
  CHECK_FALSE(r.ok);
  CHECK(r.description.empty());
  CHECK(r.attributes.empty());
  // Raw is always preserved for debugging.
  CHECK_FALSE(r.raw_json.empty());
}

TEST_CASE("ParseVlmResponse_GarbageBody_ReturnsNotOkAndDoesNotThrow",
          "[vision][vlm][protocol]") {
  auto r = parse_vlm_response("not json at all <<<");
  CHECK_FALSE(r.ok);
  CHECK(r.description.empty());
  CHECK(r.attributes.empty());
}

TEST_CASE("ParseVlmResponse_ChatEnvelopeEmptyContent_ReturnsNotOk",
          "[vision][vlm][protocol]") {
  // A valid chat envelope whose assistant content is an empty string means the
  // model returned nothing usable: it must NOT fall back to re-parsing the
  // envelope object (which would find no fields yet claim success), and it must
  // report ok==false so the stage never stores an empty annotation.
  const std::string body = R"({"choices":[{"message":{"content":""}}]})";
  auto r = parse_vlm_response(body);
  CHECK_FALSE(r.ok);
  CHECK(r.description.empty());
  CHECK(r.attributes.empty());
  // Raw is always preserved for debugging.
  CHECK_FALSE(r.raw_json.empty());
}

TEST_CASE("ParseVlmResponse_EmptyObject_ReturnsNotOk",
          "[vision][vlm][protocol]") {
  // The assistant returned a JSON object with no description and no scalar
  // fields. Recovering an object is not enough — the annotation would be empty,
  // so ok stays false (§5).
  const std::string body = R"({"choices":[{"message":{"content":"{}"}}]})";
  auto r = parse_vlm_response(body);
  CHECK_FALSE(r.ok);
  CHECK(r.description.empty());
  CHECK(r.attributes.empty());
  CHECK_FALSE(r.raw_json.empty());
}

TEST_CASE("ParseVlmResponse_NestedContainerFields_AreIgnoredNotStringified",
          "[vision][vlm][protocol]") {
  // A flat field whose value is itself an object/array is not a scalar and must
  // not become an attribute; only the scalar "material" does.
  const std::string body =
      R"({"description":"x","material":"wood","dims":{"w":1,"h":2}})";
  auto r = parse_vlm_response(body);
  REQUIRE(r.ok);
  CHECK(r.description == "x");
  REQUIRE(attr(r, "material").has_value());
  CHECK(*attr(r, "material") == "wood");
  CHECK_FALSE(attr(r, "dims").has_value());
}

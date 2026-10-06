// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the engine-build.json parser
// (vision::sam3::EngineBuildProfiles).
//
// engine-build.json is the single source of truth shared by the Python trtexec
// driver and the native C++ EngineBuilder; these tests guard the parser and the
// canonical checked-in file against drift/corruption. No GPU or TensorRT
// needed.

#include <catch2/catch_test_macros.hpp>

#include <reusex/vision/sam3/EngineBuildProfiles.hpp>

#include <filesystem>
#include <string>

using reusex::vision::sam3::EngineBuildProfiles;

namespace {

// A minimal but representative document: one fp32 static engine (vision-encoder
// rule already resolved) and one fp16 dynamic-batch engine.
constexpr const char *kSample = R"json({
  "schema_version": 1,
  "engines": {
    "vision-encoder": {
      "precision": "fp32",
      "workspace_mb": 4096,
      "shapes": {
        "images": {
          "min": [1, 3, 1008, 1008],
          "opt": [1, 3, 1008, 1008],
          "max": [1, 3, 1008, 1008]
        }
      }
    },
    "text-encoder": {
      "precision": "fp16",
      "shapes": {
        "input_ids": { "min": [1, 32], "opt": [1, 32], "max": [4, 32] }
      }
    }
  }
})json";

} // namespace

TEST_CASE("EngineBuildProfiles parses precision, workspace and shapes",
          "[vision][sam3]") {
  const auto profiles = EngineBuildProfiles::from_string(kSample);
  REQUIRE(profiles.schema_version == 1);
  REQUIRE(profiles.engines.size() == 2);

  SECTION("vision-encoder is forced fp32 with a batch-1 static image shape") {
    const auto ve = profiles.find("vision-encoder");
    REQUIRE(ve.has_value());
    CHECK(ve->is_fp32());
    CHECK_FALSE(ve->is_fp16());
    CHECK(ve->workspace_mb == 4096);
    REQUIRE(ve->shapes.count("images") == 1);
    const auto &img = ve->shapes.at("images");
    CHECK(img.min == std::vector<int>{1, 3, 1008, 1008});
    CHECK(img.opt == std::vector<int>{1, 3, 1008, 1008});
    CHECK(img.max == std::vector<int>{1, 3, 1008, 1008});
  }

  SECTION("text-encoder is fp16, dynamic batch, default workspace") {
    const auto te = profiles.find("text-encoder");
    REQUIRE(te.has_value());
    CHECK(te->is_fp16());
    CHECK(te->workspace_mb == 8192); // default when omitted
    const auto &ids = te->shapes.at("input_ids");
    CHECK(ids.min == std::vector<int>{1, 32});
    CHECK(ids.max == std::vector<int>{4, 32}); // batch grows to 4
  }

  SECTION("missing engine yields nullopt") {
    CHECK_FALSE(profiles.find("does-not-exist").has_value());
  }
}

TEST_CASE("EngineBuildProfiles rejects malformed documents", "[vision][sam3]") {
  CHECK_THROWS(EngineBuildProfiles::from_string("not json"));
  CHECK_THROWS(EngineBuildProfiles::from_string(R"({"schema_version": 2})"));
  CHECK_THROWS(EngineBuildProfiles::from_string("{}")); // no 'engines' object
  CHECK_THROWS(EngineBuildProfiles::from_string(
      R"({"engines": {"e": {"precision": "bf16"}}})")); // bad precision

  // A bare engine with no shapes and default precision is valid.
  CHECK_NOTHROW(EngineBuildProfiles::from_string(R"({"engines": {"e": {}}})"));
}

// The canonical checked-in engine-build.json must parse and carry the invariant
// that the vision-encoder is fp32 (the fp16-corruption guard) with all 8
// SAM 3.1 engines present.
TEST_CASE("canonical engine-build.json is valid and fp32-safe",
          "[vision][sam3]") {
#ifdef REUSEX_SOURCE_DIR
  const std::filesystem::path path = std::filesystem::path(REUSEX_SOURCE_DIR) /
                                     "python" / "reusex_sam3" /
                                     "engine-build.json";
  if (!std::filesystem::exists(path))
    SKIP("canonical engine-build.json not found at " + path.string());

  const auto profiles = EngineBuildProfiles::from_file(path);
  CHECK(profiles.engines.size() == 8);

  const auto ve = profiles.find("vision-encoder");
  REQUIRE(ve.has_value());
  CHECK(ve->is_fp32()); // fp16 corrupts the bf16-native ViT-L trunk

  for (const char *name :
       {"text-encoder", "geometry-encoder", "decoder", "tracker-memory-encoder",
        "tracker-memory-attention", "tracker-prompt-encoder",
        "tracker-multiplex-decoder"}) {
    INFO("engine: " << name);
    CHECK(profiles.find(name).has_value());
  }
#else
  SKIP("REUSEX_SOURCE_DIR not defined");
#endif
}

TEST_CASE("EngineBuildProfiles reads recipe_version, defaulting to 1",
          "[vision][sam3]") {
  CHECK(EngineBuildProfiles::from_string(kSample).recipe_version == 1);
  CHECK(EngineBuildProfiles::from_string(
            R"({"recipe_version": 7, "engines": {"e": {}}})")
            .recipe_version == 7);
  CHECK_THROWS(EngineBuildProfiles::from_string(
      R"({"recipe_version": 0, "engines": {"e": {}}})"));
}

TEST_CASE("EngineBuildProfiles round-trips through to_json", "[vision][sam3]") {
  const auto a = EngineBuildProfiles::from_string(kSample);
  const auto b = EngineBuildProfiles::from_string(a.to_json());
  CHECK(b.schema_version == a.schema_version);
  CHECK(b.recipe_version == a.recipe_version);
  CHECK(b.engines == a.engines);

  auto changed = b;
  changed.engines["text-encoder"].shapes["input_ids"].max = {8, 32};
  CHECK_FALSE(changed.engines == a.engines);
}

// The built-in (canonical) recipe must let box prompts reach the decoder: a
// geometry encoder that takes a single box and a decoder whose prompt_len
// leaves room for box + CLS tokens after the 32 text tokens (recipe v2).
TEST_CASE("built-in recipe supports geometry prompts", "[vision][sam3]") {
  const auto &p = EngineBuildProfiles::builtin();
  CHECK(p.recipe_version >= 2);

  const auto geo = p.find("geometry-encoder");
  REQUIRE(geo.has_value());
  const auto &boxes = geo->shapes.at("input_boxes");
  CHECK(boxes.min[1] == 1);
  CHECK(boxes.max[1] >= 1);

  const auto dec = p.find("decoder");
  REQUIRE(dec.has_value());
  const auto &pf = dec->shapes.at("prompt_features");
  CHECK(pf.min[1] == 32); // text-only annotate path unchanged
  CHECK(pf.opt[1] == 32);
  CHECK(pf.max[1] == 32 + boxes.max[1] + 1);
  CHECK(dec->shapes.at("prompt_mask").max[1] == pf.max[1]);
}

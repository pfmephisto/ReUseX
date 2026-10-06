// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the managed SAM3 model store (vision::sam3::sam3_assets):
// bundle / engine-dir completeness, the engine-recipe fallback, the status
// probe, and the staged download — driven through a local file:// manifest,
// so no network, GPU or TensorRT is needed.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/vision/sam3/sam3_assets.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
namespace sam3 = reusex::vision::sam3;
using reusex::test_support::TempDir;

namespace {

void write_file(const fs::path &p, const std::string &content) {
  fs::create_directories(p.parent_path());
  std::ofstream(p, std::ios::binary) << content;
}

std::string sha256_hex(const std::string &data) {
  unsigned char md[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  EVP_Digest(data.data(), data.size(), md, &len, EVP_sha256(), nullptr);
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  for (unsigned int i = 0; i < len; ++i) {
    out.push_back(kHex[md[i] >> 4]);
    out.push_back(kHex[md[i] & 0xf]);
  }
  return out;
}

bool contains(const std::vector<std::string> &v, const std::string &s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

/// The detector-only file set a local `make export-detector` produces — note:
/// no engine-build.json and no tracker files.
void write_detector_export(const fs::path &dir) {
  for (const auto &name : sam3::required_engines())
    write_file(dir / (name + ".onnx"), "onnx:" + name);
  write_file(dir / "tokenizer.json", "{}");
}

/// Sets (or unsets) an environment variable for the scope, then restores it.
class ScopedEnv {
    public:
  ScopedEnv(const char *name, const char *value) : name_(name) {
    if (const char *old = std::getenv(name))
      old_ = old;
    if (value)
      ::setenv(name, value, 1);
    else
      ::unsetenv(name);
  }
  ~ScopedEnv() {
    if (old_)
      ::setenv(name_, old_->c_str(), 1);
    else
      ::unsetenv(name_);
  }

    private:
  const char *name_;
  std::optional<std::string> old_;
};

/// A published-bundle source directory plus its file:// manifest.
struct BundleSource {
  explicit BundleSource(const fs::path &root) : dir(root / "release") {
    for (const auto &name : sam3::required_engines())
      add(name + ".onnx", "onnx:" + name);
    add("tokenizer.json", "{\"tok\":1}");
  }

  void add(const std::string &name, const std::string &content,
           bool optional = false, bool publish = true) {
    if (publish)
      write_file(dir / name, content);
    files.push_back({{"name", name},
                     {"sha256", sha256_hex(content)},
                     {"optional", optional}});
  }

  std::string manifest_url() const {
    write_file(dir / "manifest.json", nlohmann::json{{"files", files}}.dump(2));
    return "file://" + (dir / "manifest.json").string();
  }

  fs::path dir;
  nlohmann::json files = nlohmann::json::array();
};

sam3::Sam3AssetOptions cpu_opts(const fs::path &models_dir,
                                const std::string &manifest_url = {}) {
  sam3::Sam3AssetOptions o;
  o.models_dir = models_dir;
  o.manifest_url = manifest_url;
  o.use_cuda = false;
  return o;
}

/// No leftover staging (`onnx.partial-*`) or discarded (`onnx.stale-*`) dirs.
bool no_staging_leftovers(const fs::path &parent) {
  if (!fs::exists(parent))
    return true;
  for (const auto &e : fs::directory_iterator(parent)) {
    const auto name = e.path().filename().string();
    if (name.rfind("onnx.", 0) == 0)
      return false;
  }
  return true;
}

} // namespace

// --- completeness -----------------------------------------------------------

TEST_CASE("Sam3Assets_MissingOnnxFiles_EmptyDirNeedsDetectorSet",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const auto missing = sam3::missing_onnx_files(tmp.path / "onnx");
  CHECK(missing.size() == sam3::required_engines().size() + 1);
  CHECK(contains(missing, "vision-encoder.onnx"));
  CHECK(contains(missing, "decoder.onnx"));
  CHECK(contains(missing, "tokenizer.json"));
}

TEST_CASE("Sam3Assets_MissingOnnxFiles_LocalExportNeedsNoRecipeOrTracker",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  write_detector_export(tmp.path);
  // engine-build.json (written only by build_engines.py) and the tracker
  // graphs are not required: a bare `make export` must count as complete.
  CHECK(sam3::missing_onnx_files(tmp.path).empty());

  fs::remove(tmp.path / "geometry-encoder.onnx");
  CHECK(sam3::missing_onnx_files(tmp.path) ==
        std::vector<std::string>{"geometry-encoder.onnx"});
}

TEST_CASE("Sam3Assets_MissingOnnxFiles_PublishedBundleNeedsItsWholeManifest",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  write_detector_export(tmp.path);
  write_file(tmp.path / "bundle-manifest.json",
             R"({"files":[{"name":"vision-encoder.onnx"},
                          {"name":"LICENSE_SAM.txt"},
                          {"name":"extra.bin","optional":true}]})");
  CHECK(sam3::missing_onnx_files(tmp.path) ==
        std::vector<std::string>{"LICENSE_SAM.txt"});

  write_file(tmp.path / "LICENSE_SAM.txt", "license");
  CHECK(sam3::missing_onnx_files(tmp.path).empty());
}

TEST_CASE("Sam3Assets_MissingEngineFiles_RequiresDetectorEnginesAndTokenizer",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);

  for (const auto &name : sam3::required_engines())
    write_file(eng / (name + ".engine"), "engine");
  // All engines but no metadata (e.g. a crash before the metadata copy):
  // NOT ready.
  CHECK(sam3::missing_engine_files(onnx, eng) ==
        std::vector<std::string>{"tokenizer.json"});

  write_file(eng / "tokenizer.json", "{}");
  CHECK(sam3::missing_engine_files(onnx, eng).empty());
}

TEST_CASE("Sam3Assets_MissingEngineFiles_TrackerIsOptionalUnlessShipped",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  for (const auto &name : sam3::required_engines())
    write_file(eng / (name + ".engine"), "engine");
  write_file(eng / "tokenizer.json", "{}");
  REQUIRE(sam3::missing_engine_files(onnx, eng).empty()); // detector-only

  // A bundle that ships a tracker graph (+ its meta) needs both built/copied.
  write_file(onnx / "tracker-memory-encoder.onnx", "onnx");
  write_file(onnx / "tracker-meta.json", "{}");
  const auto missing = sam3::missing_engine_files(onnx, eng);
  CHECK(contains(missing, "tracker-memory-encoder.engine"));
  CHECK(contains(missing, "tracker-meta.json"));
  CHECK_FALSE(contains(missing, "tracker-memory-attention.engine"));
}

// --- engine recipe fallback
// ---------------------------------------------------

TEST_CASE("Sam3Assets_LoadEngineBuildProfiles_FallsBackToBuiltinRecipe",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  write_detector_export(tmp.path);

  const auto builtin = sam3::load_engine_build_profiles(tmp.path);
  CHECK(builtin.engines.size() == 8);
  for (const auto &name : sam3::required_engines())
    CHECK(builtin.find(name).has_value());
  REQUIRE(builtin.find("vision-encoder").has_value());
  CHECK(builtin.find("vision-encoder")->is_fp32()); // the fp16-corruption rule

  // An ONNX dir's own recipe wins when it is at least as new as the built-in.
  write_file(tmp.path / "engine-build.json",
             R"({"schema_version":1,"recipe_version":99,"engines":{
                 "vision-encoder":{"precision":"fp32","workspace_mb":1024}}})");
  const auto own = sam3::load_engine_build_profiles(tmp.path);
  CHECK(own.engines.size() == 1);
  CHECK(own.find("vision-encoder")->workspace_mb == 1024);
}

TEST_CASE("Sam3Assets_LoadEngineBuildProfiles_NewerBuiltinSupersedesBundle",
          "[vision][sam3][assets]") {
  // sam3.1-onnx-v1 ships a recipe_version-less (v1) engine-build.json whose
  // decoder takes text tokens only; the built-in v2 recipe must replace it so
  // managed installs get box prompts without a new ONNX release.
  TempDir tmp("sam3_assets");
  write_detector_export(tmp.path);
  write_file(tmp.path / "engine-build.json",
             R"({"schema_version":1,"engines":{"vision-encoder":
                 {"precision":"fp32","workspace_mb":1024}}})");
  const auto chosen = sam3::load_engine_build_profiles(tmp.path);
  const auto &builtin = sam3::EngineBuildProfiles::builtin();
  CHECK(chosen.recipe_version == builtin.recipe_version);
  CHECK(chosen.engines == builtin.engines);
}

// --- stale engines -----------------------------------------------------------

namespace {

/// A built engine dir for @p onnx: every required engine + tokenizer.
void write_built_engines(const fs::path &eng) {
  for (const auto &name : sam3::required_engines())
    write_file(eng / (name + ".engine"), "engine");
  write_file(eng / "tokenizer.json", "{}");
}

/// The built-in recipe re-labelled as an older version with a text-only
/// (L=32) decoder: what sam3.1-onnx-v1 shipped.
std::string old_bundle_recipe() {
  auto old = sam3::EngineBuildProfiles::builtin();
  old.recipe_version = 1;
  for (const char *in : {"prompt_features", "prompt_mask"})
    old.engines.at("decoder").shapes.at(in).max[1] = 32;
  return old.to_json();
}

} // namespace

TEST_CASE("Sam3Assets_StaleEngines_UnstampedDirComparesWithBundleRecipe",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  write_built_engines(eng);

  // Engines built before recipe stamps existed came from the bundle's own
  // recipe; only the engines whose profile changed since are stale.
  write_file(onnx / "engine-build.json", old_bundle_recipe());
  CHECK(sam3::stale_engines(onnx, eng) == std::vector<std::string>{"decoder"});
  CHECK(sam3::missing_engine_files(onnx, eng) ==
        std::vector<std::string>{"decoder.engine"});
}

TEST_CASE("Sam3Assets_StaleEngines_StampMatchingTheRecipeIsFresh",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  write_built_engines(eng);
  write_file(onnx / "engine-build.json", old_bundle_recipe());

  write_file(eng / "engine-build.json",
             sam3::EngineBuildProfiles::builtin().to_json());
  CHECK(sam3::stale_engines(onnx, eng).empty());
  CHECK(sam3::missing_engine_files(onnx, eng).empty());

  // A stamp from an older recipe marks the changed engines stale again.
  write_file(eng / "engine-build.json", old_bundle_recipe());
  CHECK(sam3::stale_engines(onnx, eng) == std::vector<std::string>{"decoder"});
}

TEST_CASE("Sam3Assets_StaleEngines_UnbuiltEnginesAreMissingNotStale",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  write_file(onnx / "engine-build.json", old_bundle_recipe());
  write_file(eng / "tokenizer.json", "{}");
  CHECK(sam3::stale_engines(onnx, eng).empty());
  CHECK(contains(sam3::missing_engine_files(onnx, eng), "decoder.engine"));
}

// --- status probe
// -------------------------------------------------------------

TEST_CASE("Sam3Assets_Status_ReportsAbsentIncompleteReadyAndNeverBuilding",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  auto opts = cpu_opts(tmp.path);

  CHECK(sam3::sam3_status(opts).state == sam3::PrepState::absent);

  const fs::path onnx = sam3::sam3_onnx_dir(opts);
  write_file(onnx / "vision-encoder.onnx", "partial");
  const auto partial = sam3::sam3_status(opts);
  CHECK(partial.state == sam3::PrepState::absent);
  CHECK(partial.message.find("incomplete") != std::string::npos);

  write_detector_export(onnx);
  CHECK(sam3::sam3_status(opts).state == sam3::PrepState::ready);

  // CUDA with no engines: a pure probe knows nothing is building, so it must
  // not claim "building" (pollers would wait forever).
  opts.use_cuda = true;
  const auto cuda = sam3::sam3_status(opts).state;
  CHECK(cuda != sam3::PrepState::building);
  CHECK((cuda == sam3::PrepState::not_built || cuda == sam3::PrepState::error));
  CHECK(std::string(sam3::to_string(sam3::PrepState::not_built)) ==
        "not_built");
}

// --- staged download (file:// manifest)
// ---------------------------------------

TEST_CASE("Sam3Assets_Prepare_DownloadsVerifiesAndPublishesAtomically",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  BundleSource src(tmp.path);
  src.add("engine-build.json.absent-optional", "x", /*optional=*/true,
          /*publish=*/false);
  const auto opts = cpu_opts(tmp.path / "models", src.manifest_url());

  std::vector<sam3::PrepState> states;
  const fs::path dir = sam3::prepare_sam3_model(
      opts, [&](const sam3::PrepProgress &p) { states.push_back(p.state); });

  CHECK(dir == sam3::sam3_onnx_dir(opts));
  CHECK(sam3::missing_onnx_files(dir).empty());
  CHECK(fs::exists(dir / "bundle-manifest.json"));
  CHECK_FALSE(fs::exists(dir / "engine-build.json.absent-optional"));
  CHECK(no_staging_leftovers(dir.parent_path()));
  REQUIRE_FALSE(states.empty());
  CHECK(states.front() == sam3::PrepState::downloading);
  CHECK(states.back() == sam3::PrepState::ready);
  CHECK(sam3::sam3_status(opts).state == sam3::PrepState::ready);
}

TEST_CASE("Sam3Assets_Prepare_ShaMismatchPublishesNothing",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  BundleSource src(tmp.path);
  const auto url = src.manifest_url();
  write_file(src.dir / "decoder.onnx", "tampered"); // hash no longer matches

  const auto opts = cpu_opts(tmp.path / "models", url);
  CHECK_THROWS_WITH(sam3::prepare_sam3_model(opts),
                    Catch::Matchers::ContainsSubstring("sha256 mismatch"));
  CHECK_FALSE(fs::exists(sam3::sam3_onnx_dir(opts)));
  CHECK(no_staging_leftovers(sam3::sam3_onnx_dir(opts).parent_path()));
}

TEST_CASE("Sam3Assets_Prepare_InterruptedDownloadIsRetriedNotTrusted",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  BundleSource src(tmp.path);
  src.add("LICENSE_SAM.txt", "license", /*optional=*/false, /*publish=*/false);
  const auto opts = cpu_opts(tmp.path / "models", src.manifest_url());

  // First attempt: a required file is unavailable → nothing published.
  CHECK_THROWS(sam3::prepare_sam3_model(opts));
  CHECK_FALSE(fs::exists(sam3::sam3_onnx_dir(opts)));
  CHECK(sam3::sam3_status(opts).state == sam3::PrepState::absent);

  // A stale half-populated dir from an older, non-staged download must not
  // count as complete either, and is replaced by the retry.
  write_file(sam3::sam3_onnx_dir(opts) / "vision-encoder.onnx", "stale");
  write_file(sam3::sam3_onnx_dir(opts) / "bundle-manifest.json",
             nlohmann::json{{"files", src.files}}.dump());
  CHECK_FALSE(sam3::missing_onnx_files(sam3::sam3_onnx_dir(opts)).empty());

  write_file(src.dir / "LICENSE_SAM.txt", "license"); // now published
  const fs::path dir = sam3::prepare_sam3_model(opts);
  CHECK(sam3::missing_onnx_files(dir).empty());
  CHECK(fs::exists(dir / "LICENSE_SAM.txt"));
  CHECK(no_staging_leftovers(dir.parent_path()));
}

TEST_CASE("Sam3Assets_Prepare_ConcurrentCallersShareOneDownload",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  BundleSource src(tmp.path);
  const auto opts = cpu_opts(tmp.path / "models", src.manifest_url());

  // Stands in for the CPU and CUDA provisioning slots resolving the same ONNX
  // dir at once.
  std::vector<std::thread> threads;
  std::atomic<int> ok{0};
  for (int i = 0; i < 4; ++i)
    threads.emplace_back([&] {
      try {
        if (sam3::prepare_sam3_model(opts) == sam3::sam3_onnx_dir(opts))
          ++ok;
      } catch (...) {
      }
    });
  for (auto &t : threads)
    t.join();

  CHECK(ok == 4);
  CHECK(sam3::missing_onnx_files(sam3::sam3_onnx_dir(opts)).empty());
  CHECK(no_staging_leftovers(sam3::sam3_onnx_dir(opts).parent_path()));
}

TEST_CASE("Sam3Assets_Prepare_RaisedCancelFlagAborts",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  BundleSource src(tmp.path);
  auto opts = cpu_opts(tmp.path / "models", src.manifest_url());
  std::atomic<bool> cancel{true};
  opts.cancel = &cancel;

  CHECK_THROWS_AS(sam3::prepare_sam3_model(opts), sam3::Sam3PrepCancelled);
  CHECK_FALSE(fs::exists(sam3::sam3_onnx_dir(opts)));
}

TEST_CASE("Sam3Assets_Prepare_IncompleteOnnxDirOverrideIsHardError",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path exported = tmp.path / "my-export";
  write_detector_export(exported);
  fs::remove(exported / "tokenizer.json");
  ScopedEnv override_dir("REUSEX_SAM3_ONNX_DIR", exported.c_str());

  BundleSource src(tmp.path);
  const auto opts = cpu_opts(tmp.path / "models", src.manifest_url());
  // A user-provided export is never downloaded into; the error names the gap.
  CHECK_THROWS_WITH(sam3::prepare_sam3_model(opts),
                    Catch::Matchers::ContainsSubstring("tokenizer.json"));
  CHECK_FALSE(fs::exists(exported / "bundle-manifest.json"));

  // Complete it (still without engine-build.json): the CPU path is ready.
  write_file(exported / "tokenizer.json", "{}");
  CHECK(sam3::prepare_sam3_model(opts) == exported);
}

TEST_CASE("Sam3Assets_FallbackEngines_ReadFromTheStampAndLoadable",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  write_built_engines(eng);
  CHECK(sam3::fallback_engines(eng).empty()); // no stamp

  // A stamp recording a text-only fallback build: the engine is loadable
  // (not stale, not missing) but listed so preparation retries its profile.
  auto stamp = sam3::EngineBuildProfiles::builtin();
  stamp.fallback_engines = {"decoder"};
  write_file(eng / "engine-build.json", stamp.to_json());
  CHECK(sam3::fallback_engines(eng) == std::vector<std::string>{"decoder"});
  CHECK(sam3::stale_engines(onnx, eng).empty());
  CHECK(sam3::missing_engine_files(onnx, eng).empty());

  write_file(eng / "engine-build.json", "not json");
  CHECK(sam3::fallback_engines(eng).empty());
}

TEST_CASE("Sam3Assets_FallbackEngines_RetriedOnlyWhenTheOnnxChanges",
          "[vision][sam3][assets]") {
  TempDir tmp("sam3_assets");
  const fs::path onnx = tmp.path / "onnx";
  const fs::path eng = tmp.path / "engines";
  write_detector_export(onnx);
  write_built_engines(eng);

  auto stamp = sam3::EngineBuildProfiles::builtin();
  stamp.fallback_engines = {"decoder"};

  SECTION("a stamp from before fingerprints is retried once") {
    write_file(eng / "engine-build.json", stamp.to_json());
    CHECK(sam3::fallback_engines_to_retry(onnx, eng) ==
          std::vector<std::string>{"decoder"});
  }
  SECTION("a fingerprint matching the ONNX makes the fallback permanent") {
    stamp.fallback_onnx_sha256["decoder"] = sha256_hex("onnx:decoder");
    write_file(eng / "engine-build.json", stamp.to_json());
    CHECK(sam3::fallback_engines_to_retry(onnx, eng).empty());
    // Still recorded as a fallback (the loader names it), still loadable.
    CHECK(sam3::fallback_engines(eng) == std::vector<std::string>{"decoder"});
    CHECK(sam3::missing_engine_files(onnx, eng).empty());

    // A re-export (say, with dynamic axes) changes the ONNX: retry.
    write_file(onnx / "decoder.onnx", "onnx:decoder, re-exported");
    CHECK(sam3::fallback_engines_to_retry(onnx, eng) ==
          std::vector<std::string>{"decoder"});
  }
  SECTION("a changed recipe rebuilds the engine as stale, not as a retry") {
    stamp.fallback_onnx_sha256["decoder"] = sha256_hex("onnx:decoder");
    for (const char *in : {"prompt_features", "prompt_mask"})
      stamp.engines.at("decoder").shapes.at(in).max[1] = 32;
    write_file(eng / "engine-build.json", stamp.to_json());
    CHECK(sam3::fallback_engines_to_retry(onnx, eng).empty());
    CHECK(sam3::stale_engines(onnx, eng) ==
          std::vector<std::string>{"decoder"});
  }
  SECTION("a deleted engine is missing, the manual way to force a rebuild") {
    stamp.fallback_onnx_sha256["decoder"] = sha256_hex("onnx:decoder");
    write_file(eng / "engine-build.json", stamp.to_json());
    fs::remove(eng / "decoder.engine");
    CHECK(sam3::fallback_engines_to_retry(onnx, eng).empty());
    CHECK(contains(sam3::missing_engine_files(onnx, eng), "decoder.engine"));
  }
}

TEST_CASE("Sam3Assets_Status_TellsARecipeUpdateFromAFirstBuild",
          "[vision][sam3][assets]") {
  ScopedEnv no_override("REUSEX_SAM3_ONNX_DIR", nullptr);
  TempDir tmp("sam3_assets");
  auto opts = cpu_opts(tmp.path);
  opts.use_cuda = true;
  const fs::path onnx = sam3::sam3_onnx_dir(opts);
  write_detector_export(onnx);
  if (sam3::sam3_status(opts).state == sam3::PrepState::error)
    SKIP("no TensorRT backend in this build");

  // First build: nothing built yet.
  CHECK(sam3::sam3_status(opts).update_engines.empty());

  // Engines from the v1 recipe: a one-time rebuild of the changed ones.
  const fs::path eng = sam3::sam3_engine_dir(opts);
  write_built_engines(eng);
  write_file(eng / "engine-build.json", old_bundle_recipe());
  const auto update = sam3::sam3_status(opts);
  CHECK(update.state == sam3::PrepState::not_built);
  CHECK(update.update_engines == std::vector<std::string>{"decoder"});

  // An engine that is really missing makes it a build, not an update.
  fs::remove(eng / "vision-encoder.engine");
  const auto build = sam3::sam3_status(opts);
  CHECK(build.state == sam3::PrepState::not_built);
  CHECK(build.update_engines.empty());
}

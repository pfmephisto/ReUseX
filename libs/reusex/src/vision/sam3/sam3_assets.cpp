// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/sam3/sam3_assets.hpp"
#include "core/logging.hpp"
#include "vision/sam3/EngineBuildProfiles.hpp"

#ifdef REUSEX_USE_TENSORRT
#include "vision/tensor_rt/common/EngineBuilder.hpp"

#include <NvInferVersion.h>
#include <cuda_runtime.h>
#endif

#include <curl/curl.h>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

namespace reusex::vision::sam3 {

namespace fs = std::filesystem;

// The managed model directory name (SAM 3.1 uses the SAM 3 detector 1:1 plus a
// new tracker; the on-disk layout is shared).
static constexpr const char *kSubDir = "sam3.1";

// Default release manifest for the managed SAM3 model bundle. The bundle is a
// derivative of Meta's SAM checkpoint, redistributed under the SAM License
// (LICENSE_SAM.txt ships in the release; see docs/sam3.1-tensorrt.md §9).
//
// sam3.1-onnx-v1 is DETECTOR-ONLY (image + panorama segmentation): the 4
// detector ONNX + tokenizer.json + engine-build.json. The SAM 3.1 video-tracker
// engines are not in v1; a future bundle version adds them (the C++ builder
// already skips engines whose ONNX is absent). Override with a full local
// export via REUSEX_SAM3_ONNX_DIR, or a different bundle via
// --sam3-manifest-url.
static constexpr const char *kDefaultManifestUrl =
    "https://github.com/pfmephisto/ReUseX/releases/download/sam3.1-onnx-v1/"
    "manifest.json";

const char *to_string(PrepState state) {
  switch (state) {
  case PrepState::absent:
    return "absent";
  case PrepState::not_built:
    return "not_built";
  case PrepState::downloading:
    return "downloading";
  case PrepState::building:
    return "building";
  case PrepState::ready:
    return "ready";
  case PrepState::error:
    return "error";
  }
  return "unknown";
}

namespace {

// Where a downloaded bundle records the manifest it was verified against. Its
// presence marks the directory as a published (complete, verified) download.
constexpr const char *kBundleManifest = "bundle-manifest.json";
constexpr const char *kTokenizer = "tokenizer.json";
constexpr const char *kTrackerMeta = "tracker-meta.json";
constexpr const char *kEngineBuild = "engine-build.json";

std::optional<std::string> env(const char *name) {
  const char *v = std::getenv(name);
  if (v && *v)
    return std::string(v);
  return std::nullopt;
}

fs::path default_cache_root() {
  if (auto xdg = env("XDG_CACHE_HOME"))
    return fs::path(*xdg) / "reusex" / "models";
  if (auto home = env("HOME"))
    return fs::path(*home) / ".cache" / "reusex" / "models";
  return fs::temp_directory_path() / "reusex" / "models";
}

void report(const ProgressCallback &cb, PrepState state, float frac,
            std::string msg) {
  if (cb)
    cb(PrepProgress{state, frac, std::move(msg)});
}

bool is_cancelled(const std::atomic<bool> *cancel) {
  return cancel && cancel->load(std::memory_order_relaxed);
}

void throw_if_cancelled(const std::atomic<bool> *cancel, const char *where) {
  if (is_cancelled(cancel))
    throw Sam3PrepCancelled(
        fmt::format("SAM3 model preparation cancelled ({})", where));
}

bool is_required_engine(const std::string &name) {
  const auto &req = required_engines();
  return std::find(req.begin(), req.end(), name) != req.end();
}

/// Unique sibling path of @p dir for staging / discarding (pid + random salt,
/// so neither two threads nor two processes ever share one).
fs::path unique_sibling(const fs::path &dir, const char *tag) {
  static std::atomic<unsigned> counter{0};
  static const unsigned salt = std::random_device{}();
  return dir.parent_path() /
         fmt::format("{}.{}-{}-{:x}-{}", dir.filename().string(), tag,
                     static_cast<long>(::getpid()), salt, counter++);
}

std::string join(const std::vector<std::string> &v) {
  std::string out;
  for (const auto &s : v)
    out += (out.empty() ? "" : ", ") + s;
  return out;
}

// --- sha256 (OpenSSL EVP) --------------------------------------------------

std::string to_hex(const unsigned char *data, unsigned int len) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (unsigned int i = 0; i < len; ++i) {
    out.push_back(kHex[data[i] >> 4]);
    out.push_back(kHex[data[i] & 0xf]);
  }
  return out;
}

std::string sha256_file(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    throw std::runtime_error(
        fmt::format("sha256: cannot open {}", path.string()));

  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx)
    throw std::runtime_error("sha256: EVP_MD_CTX_new failed");
  struct CtxGuard {
    EVP_MD_CTX *c;
    ~CtxGuard() { EVP_MD_CTX_free(c); }
  } guard{ctx};

  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("sha256: DigestInit failed");

  std::array<char, 1 << 16> buf;
  while (in) {
    in.read(buf.data(), buf.size());
    const std::streamsize n = in.gcount();
    if (n > 0 && EVP_DigestUpdate(ctx, buf.data(), static_cast<size_t>(n)) != 1)
      throw std::runtime_error("sha256: DigestUpdate failed");
  }

  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &len) != 1)
    throw std::runtime_error("sha256: DigestFinal failed");
  return to_hex(digest, len);
}

// --- libcurl download ------------------------------------------------------

void ensure_curl_global() {
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

size_t write_to_file(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *out = static_cast<std::ofstream *>(userdata);
  out->write(ptr, static_cast<std::streamsize>(size * nmemb));
  return out->good() ? size * nmemb : 0;
}

/// libcurl progress hook: a non-zero return aborts the transfer, which is how
/// a raised cancel flag interrupts a multi-GB download mid-stream.
int abort_if_cancelled(void *userdata, curl_off_t, curl_off_t, curl_off_t,
                       curl_off_t) {
  return is_cancelled(static_cast<const std::atomic<bool> *>(userdata)) ? 1 : 0;
}

/// Download `url` to `dest` (via `dest.part` + rename). Returns false when the
/// resource does not exist (HTTP 404, or a missing file:// path) so the caller
/// decides whether that is fatal; throws on any other transport/HTTP error and
/// Sam3PrepCancelled when @p cancel is raised mid-transfer.
bool download_to(const std::string &url, const fs::path &dest,
                 const std::atomic<bool> *cancel) {
  ensure_curl_global();
  fs::create_directories(dest.parent_path());
  const fs::path tmp = dest.string() + ".part";

  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("download: curl_easy_init failed");
  struct CurlGuard {
    CURL *c;
    ~CurlGuard() { curl_easy_cleanup(c); }
  } cguard{curl};

  std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
  if (!out.is_open())
    throw std::runtime_error(
        fmt::format("download: cannot open {} for writing", tmp.string()));

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 0L); // inspect code manually
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_file);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, abort_if_cancelled);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA,
                   const_cast<std::atomic<bool> *>(cancel));
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "reusex-sam3/1.0");

  const CURLcode res = curl_easy_perform(curl);
  out.close();

  auto discard = [&] {
    std::error_code ec;
    fs::remove(tmp, ec);
  };

  if (res == CURLE_ABORTED_BY_CALLBACK) {
    discard();
    throw Sam3PrepCancelled(
        fmt::format("SAM3 model preparation cancelled (downloading {})", url));
  }
  if (res == CURLE_FILE_COULDNT_READ_FILE) { // file:// resource absent
    discard();
    return false;
  }
  if (res != CURLE_OK) {
    discard();
    throw std::runtime_error(
        fmt::format("download: {} failed: {}", url, curl_easy_strerror(res)));
  }

  long code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
  if (code == 404) {
    discard();
    return false;
  }
  if (code >= 400) {
    discard();
    throw std::runtime_error(
        fmt::format("download: {} returned HTTP {}", url, code));
  }

  fs::rename(tmp, dest);
  return true;
}

// --- manifest --------------------------------------------------------------

struct ManifestFile {
  std::string name;
  std::string url;    // absolute; resolved from base_url + name if unset
  std::string sha256; // optional (empty → skip verification)
  bool optional = false;
};

struct Manifest {
  std::string base_url;
  std::vector<ManifestFile> files;
};

/// A manifest file name must be a plain file name inside the bundle dir.
void check_file_name(const std::string &name) {
  const fs::path p(name);
  if (name.empty() || p.has_parent_path() || p.is_absolute() || name == "." ||
      name == ".." || name == kBundleManifest)
    throw std::runtime_error(fmt::format(
        "SAM3 manifest: invalid file name '{}' (must be a plain file name)",
        name));
}

Manifest parse_manifest(const fs::path &path, const std::string &manifest_url) {
  std::ifstream in(path);
  if (!in.is_open())
    throw std::runtime_error(
        fmt::format("SAM3 manifest: cannot open {}", path.string()));
  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception &e) {
    throw std::runtime_error(fmt::format("SAM3 manifest: parse error in {}: {}",
                                         path.string(), e.what()));
  }

  Manifest m;
  m.base_url = j.value("base_url", std::string{});
  if (m.base_url.empty() && !manifest_url.empty()) {
    // Default: files sit next to the manifest.
    const auto slash = manifest_url.rfind('/');
    if (slash != std::string::npos)
      m.base_url = manifest_url.substr(0, slash + 1);
  }
  auto files = j.find("files");
  if (files == j.end() || !files->is_array())
    throw std::runtime_error("SAM3 manifest: missing 'files' array");
  for (const auto &f : *files) {
    ManifestFile mf;
    mf.name = f.at("name").get<std::string>();
    check_file_name(mf.name);
    mf.url = f.value("url", std::string{});
    mf.sha256 = f.value("sha256", std::string{});
    mf.optional = f.value("optional", false);
    m.files.push_back(std::move(mf));
  }
  return m;
}

/// Every download is serialized process-wide: the CPU and CUDA provisioning
/// paths resolve the same ONNX dir, and even with private staging dirs there is
/// no point fetching the same multi-GB bundle twice.
std::mutex &download_mutex() {
  static std::mutex m;
  return m;
}

/// Removes a directory tree on scope exit unless released.
struct DirGuard {
  fs::path dir;
  ~DirGuard() {
    if (dir.empty())
      return;
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
};

/// Download the bundle into a private staging dir, verify it, and publish it
/// at @p onnx_dir by rename. Never leaves a half-populated @p onnx_dir.
void download_bundle(const std::string &manifest_url, const fs::path &onnx_dir,
                     const ProgressCallback &cb,
                     const std::atomic<bool> *cancel) {
  report(cb, PrepState::downloading, 0.0f, "fetching manifest");
  fs::create_directories(onnx_dir.parent_path());
  DirGuard staging{unique_sibling(onnx_dir, "partial")};
  fs::create_directories(staging.dir);

  const fs::path manifest_path = staging.dir / kBundleManifest;
  if (!download_to(manifest_url, manifest_path, cancel))
    throw std::runtime_error(
        fmt::format("SAM3 manifest not found: {}", manifest_url));
  const Manifest m = parse_manifest(manifest_path, manifest_url);
  if (m.files.empty())
    throw std::runtime_error(
        fmt::format("SAM3 manifest {} lists no files", manifest_url));

  const std::size_t n = m.files.size();
  for (std::size_t i = 0; i < n; ++i) {
    throw_if_cancelled(cancel, "download");
    const auto &f = m.files[i];
    const fs::path dest = staging.dir / f.name;
    const std::string url = !f.url.empty() ? f.url : (m.base_url + f.name);

    report(cb, PrepState::downloading, float(i) / float(n),
           fmt::format("downloading {} ({}/{})", f.name, i + 1, n));
    reusex::info("SAM3 assets: downloading {} → {}", url, dest.string());

    if (!download_to(url, dest, cancel)) {
      if (f.optional) {
        reusex::info("SAM3 assets: optional file {} not published, skipping",
                     f.name);
        continue;
      }
      throw std::runtime_error(
          fmt::format("SAM3 assets: required file missing (404): {}", url));
    }
    if (!f.sha256.empty()) {
      const std::string got_hash = sha256_file(dest);
      if (got_hash != f.sha256)
        throw std::runtime_error(
            fmt::format("SAM3 assets: sha256 mismatch for {}:\n  expected {}\n "
                        " got      {}",
                        f.name, f.sha256, got_hash));
    } else {
      reusex::warn("SAM3 assets: manifest carries no sha256 for {}; "
                   "integrity not verified",
                   f.name);
    }
  }

  const auto missing = missing_onnx_files(staging.dir);
  if (!missing.empty())
    throw std::runtime_error(
        fmt::format("SAM3 download from {} is incomplete; missing: {}",
                    manifest_url, join(missing)));

  // Publish. rename() onto an absent path is atomic, so a concurrent process
  // either sees no bundle or a complete one.
  std::error_code ec;
  if (fs::exists(onnx_dir)) {
    if (missing_onnx_files(onnx_dir).empty()) {
      reusex::info("SAM3 assets: {} was completed concurrently; discarding "
                   "this download",
                   onnx_dir.string());
      return; // DirGuard removes our staging copy
    }
    // A stale, incomplete dir (e.g. left by an older interrupted download):
    // move it aside first so the publish below is a rename onto nothing.
    const fs::path stale = unique_sibling(onnx_dir, "stale");
    reusex::warn("SAM3 assets: replacing incomplete bundle at {} (missing: {})",
                 onnx_dir.string(), join(missing_onnx_files(onnx_dir)));
    // Another process may have moved it aside already; the publish below then
    // sorts out who won.
    fs::rename(onnx_dir, stale, ec);
    DirGuard stale_guard{ec ? fs::path{} : stale};
    ec.clear();
  }
  fs::rename(staging.dir, onnx_dir, ec);
  if (ec) {
    // Lost a cross-process race: fine if the winner published a whole bundle.
    if (fs::exists(onnx_dir) && missing_onnx_files(onnx_dir).empty()) {
      reusex::info("SAM3 assets: another process published {} first",
                   onnx_dir.string());
      return;
    }
    throw std::runtime_error(fmt::format("SAM3 assets: cannot publish {}: {}",
                                         onnx_dir.string(), ec.message()));
  }
  staging.dir.clear(); // published; nothing to clean up
  report(cb, PrepState::downloading, 1.0f, "download complete");
}

/// Names of the optional (tracker) engines whose ONNX is present.
std::vector<std::string>
present_optional_engines(const fs::path &onnx_dir,
                         const EngineBuildProfiles &profiles) {
  std::vector<std::string> out;
  for (const auto &[name, _] : profiles.engines)
    if (!is_required_engine(name) && fs::exists(onnx_dir / (name + ".onnx")))
      out.push_back(name);
  return out;
}

#ifdef REUSEX_USE_TENSORRT
std::string sanitize(std::string s) {
  for (char &c : s)
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '.' && c != '-')
      c = '_';
  return s;
}

/// Device+TensorRT key so engines built for one GPU/TRT never load on another.
std::string engine_cache_key() {
  cudaDeviceProp prop{};
  int device = 0;
  std::string gpu = "unknown-gpu";
  int cc_major = 0, cc_minor = 0;
  if (cudaGetDevice(&device) == cudaSuccess &&
      cudaGetDeviceProperties(&prop, device) == cudaSuccess) {
    gpu = prop.name;
    cc_major = prop.major;
    cc_minor = prop.minor;
  }
  return sanitize(fmt::format("{}-sm{}{}-trt{}.{}.{}", gpu, cc_major, cc_minor,
                              NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR,
                              NV_TENSORRT_PATCH));
}

/// Copy @p from to @p to via a temp file + rename, so a crash
/// never leaves a truncated copy that later passes a presence check.
void copy_atomic(const fs::path &from, const fs::path &to) {
  const fs::path tmp = unique_sibling(to, "part");
  fs::copy_file(from, tmp, fs::copy_options::overwrite_existing);
  fs::rename(tmp, to);
}

void build_engines(const fs::path &onnx_dir, const fs::path &engine_dir,
                   const ProgressCallback &cb,
                   const std::atomic<bool> *cancel) {
  const auto profiles = load_engine_build_profiles(onnx_dir);
  fs::create_directories(engine_dir);

  // Metadata FIRST: engines_ready() also checks it, so a crash after the last
  // .engine can no longer leave a "complete-looking" but unloadable dir.
  copy_atomic(onnx_dir / kTokenizer, engine_dir / kTokenizer);
  const auto optional = present_optional_engines(onnx_dir, profiles);
  if (!optional.empty() && fs::exists(onnx_dir / kTrackerMeta))
    copy_atomic(onnx_dir / kTrackerMeta, engine_dir / kTrackerMeta);

  std::vector<std::string> to_build;
  std::vector<std::string> skipped;
  for (const auto &name : required_engines()) {
    if (!profiles.find(name))
      throw std::runtime_error(fmt::format(
          "SAM3 engine build: recipe has no entry for required engine '{}'",
          name));
    if (!fs::exists(onnx_dir / (name + ".onnx")))
      throw std::runtime_error(
          fmt::format("SAM3 engine build: required ONNX {} is missing",
                      (onnx_dir / (name + ".onnx")).string()));
    to_build.push_back(name);
  }
  for (const auto &[name, _] : profiles.engines) {
    if (is_required_engine(name))
      continue;
    if (fs::exists(onnx_dir / (name + ".onnx")))
      to_build.push_back(name);
    else
      skipped.push_back(name);
  }
  if (!skipped.empty())
    reusex::info("SAM3 engine build: optional graphs not in this bundle, "
                 "skipping: {}",
                 join(skipped));

  // Engines built from an older recipe are rebuilt in place (the builder
  // writes atomically, so the old engine stays loadable until replaced).
  const auto stale = stale_engines(onnx_dir, engine_dir);
  if (!stale.empty())
    reusex::info("SAM3 engine build: recipe changed, rebuilding: {}",
                 join(stale));
  // Engines an earlier build could only make text-only: try the recipe again.
  const auto retry = fallback_engines(engine_dir);
  if (!retry.empty())
    reusex::info("SAM3 engine build: retrying the geometry-prompt profile "
                 "for the text-only fallback build(s): {}",
                 join(retry));
  auto listed = [](const std::vector<std::string> &v, const std::string &x) {
    return std::find(v.begin(), v.end(), x) != v.end();
  };

  std::vector<std::string> built_fallback;
  const std::size_t n = to_build.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::string &name = to_build[i];
    const fs::path engine = engine_dir / (name + ".engine");
    if (fs::exists(engine) && !listed(stale, name) && !listed(retry, name))
      continue; // already built for this device/TRT and recipe
    throw_if_cancelled(cancel, "engine build");
    report(cb, PrepState::building, float(i) / float(n),
           fmt::format("building {} engine ({}/{})", name, i + 1, n));

    tensor_rt::EngineBuildRequest req;
    req.onnx_path = onnx_dir / (name + ".onnx");
    req.engine_path = engine; // EngineBuilder writes atomically
    req.profile = *profiles.find(name);
    if (fs::exists(engine) && listed(retry, name) && !listed(stale, name)) {
      // A retry of a text-only fallback build: the fallback engine on disk
      // stays in use (the builder writes atomically) unless the recipe
      // profile now builds, whatever the reason it does not.
      try {
        tensor_rt::build_engine(req);
        reusex::info("SAM3 engine build: {} now builds with the "
                     "geometry-prompt profile",
                     name);
      } catch (const std::exception &e) {
        reusex::warn("SAM3 engine build: {} still fails with the "
                     "geometry-prompt profile ({}); keeping the text-only "
                     "build",
                     name, e.what());
        built_fallback.push_back(name);
      }
      continue;
    }
    try {
      tensor_rt::build_engine(req);
    } catch (const tensor_rt::EngineProfileError &e) {
      // Only a shape error: an export that bakes the prompt length / box
      // count cannot take the geometry profile, so build it text-only and
      // text segmentation survives. Any other failure (OOM, disk) is not
      // the export's fault and propagates, so it is retried as a whole.
      const auto fallback = text_only_fallback(name, req.profile);
      if (!fallback || *fallback == req.profile)
        throw;
      reusex::warn("SAM3 engine build: {} failed with the geometry-prompt "
                   "profile ({}); building it text-only instead — box and "
                   "point prompts are unavailable until a later preparation "
                   "builds the profile",
                   name, e.what());
      throw_if_cancelled(cancel, "engine build");
      req.profile = *fallback;
      tensor_rt::build_engine(req);
      built_fallback.push_back(name);
    }
  }

  // Stamp LAST: it vouches that every engine matches this recipe, so a crash
  // mid-build leaves the old (or no) stamp and the next run rebuilds. A
  // text-only fallback build is recorded as such, so the next preparation
  // retries its recipe profile and the loader can say why geometry is off.
  {
    EngineBuildProfiles stamped = profiles;
    stamped.fallback_engines = built_fallback;
    const fs::path stamp = engine_dir / kEngineBuild;
    const fs::path tmp = unique_sibling(stamp, "part");
    std::ofstream(tmp) << stamped.to_json() << '\n';
    fs::rename(tmp, stamp);
  }

  const auto missing = missing_engine_files(onnx_dir, engine_dir);
  if (!missing.empty())
    throw std::runtime_error(
        fmt::format("SAM3 engine build finished but {} still lacks: {}",
                    engine_dir.string(), join(missing)));
  report(cb, PrepState::building, 1.0f, "engine build complete");
}
#endif // REUSEX_USE_TENSORRT

} // namespace

const std::vector<std::string> &required_engines() {
  static const std::vector<std::string> engines{
      "vision-encoder", "text-encoder", "geometry-encoder", "decoder"};
  return engines;
}

EngineBuildProfiles load_engine_build_profiles(const fs::path &onnx_dir) {
  const fs::path own = onnx_dir / kEngineBuild;
  const auto &builtin = EngineBuildProfiles::builtin();
  if (fs::exists(own)) {
    auto profiles = EngineBuildProfiles::from_file(own);
    if (profiles.recipe_version >= builtin.recipe_version)
      return profiles;
    reusex::debug("SAM3: {} carries recipe v{}; the built-in recipe v{} "
                  "supersedes it",
                  own.string(), profiles.recipe_version,
                  builtin.recipe_version);
    return builtin;
  }
  reusex::debug("SAM3: {} has no {}; using the built-in canonical recipe",
                onnx_dir.string(), kEngineBuild);
  return builtin;
}

std::vector<std::string> stale_engines(const fs::path &onnx_dir,
                                       const fs::path &engine_dir) {
  const auto current = load_engine_build_profiles(onnx_dir);

  // What the engines on disk were built from: the dir's stamp, else (a dir
  // built before stamps) the bundle's own recipe, else assume current.
  std::optional<EngineBuildProfiles> built_with;
  for (const fs::path &p :
       {engine_dir / kEngineBuild, onnx_dir / kEngineBuild}) {
    if (!fs::exists(p))
      continue;
    try {
      built_with = EngineBuildProfiles::from_file(p);
    } catch (const std::exception &e) {
      reusex::warn("SAM3: unreadable engine recipe {} ({}); rebuilding",
                   p.string(), e.what());
      built_with = EngineBuildProfiles{}; // empty: every engine is stale
    }
    break;
  }
  if (!built_with)
    return {};

  std::vector<std::string> stale;
  for (const auto &[name, profile] : current.engines) {
    if (!fs::exists(engine_dir / (name + ".engine")))
      continue;
    const auto was = built_with->find(name);
    if (!was || !(*was == profile))
      stale.push_back(name);
  }
  return stale;
}

std::vector<std::string> fallback_engines(const fs::path &engine_dir) {
  const fs::path stamp = engine_dir / kEngineBuild;
  if (!fs::exists(stamp))
    return {};
  try {
    return EngineBuildProfiles::from_file(stamp).fallback_engines;
  } catch (const std::exception &) {
    return {}; // stale_engines() reports and rebuilds an unreadable stamp
  }
}

std::vector<std::string> missing_onnx_files(const fs::path &onnx_dir) {
  std::set<std::string> required;
  for (const auto &name : required_engines())
    required.insert(name + ".onnx");
  required.insert(kTokenizer);

  // A published download must additionally hold everything its manifest
  // lists as non-optional.
  const fs::path manifest = onnx_dir / kBundleManifest;
  if (fs::exists(manifest)) {
    try {
      for (const auto &f : parse_manifest(manifest, "").files)
        if (!f.optional)
          required.insert(f.name);
    } catch (const std::exception &e) {
      reusex::warn("SAM3: unreadable {} ({}); treating bundle as incomplete",
                   manifest.string(), e.what());
      return {kBundleManifest};
    }
  }

  std::vector<std::string> missing;
  for (const auto &name : required)
    if (!fs::exists(onnx_dir / name))
      missing.push_back(name);
  return missing;
}

std::vector<std::string> missing_engine_files(const fs::path &onnx_dir,
                                              const fs::path &engine_dir) {
  std::vector<std::string> missing;
  EngineBuildProfiles profiles;
  try {
    profiles = load_engine_build_profiles(onnx_dir);
  } catch (const std::exception &e) {
    reusex::warn("SAM3: cannot read engine recipe for {}: {}",
                 onnx_dir.string(), e.what());
    return {kEngineBuild};
  }

  const auto stale = stale_engines(onnx_dir, engine_dir);
  auto absent_or_stale = [&](const std::string &name) {
    return !fs::exists(engine_dir / (name + ".engine")) ||
           std::find(stale.begin(), stale.end(), name) != stale.end();
  };
  for (const auto &name : required_engines())
    if (absent_or_stale(name))
      missing.push_back(name + ".engine");
  const auto optional = present_optional_engines(onnx_dir, profiles);
  for (const auto &name : optional)
    if (absent_or_stale(name))
      missing.push_back(name + ".engine");

  if (!fs::exists(engine_dir / kTokenizer))
    missing.push_back(kTokenizer);
  if (!optional.empty() && fs::exists(onnx_dir / kTrackerMeta) &&
      !fs::exists(engine_dir / kTrackerMeta))
    missing.push_back(kTrackerMeta);
  return missing;
}

fs::path resolve_models_dir(const fs::path &explicit_dir) {
  if (!explicit_dir.empty())
    return explicit_dir;
  if (auto e = env("REUSEX_MODELS_DIR"))
    return fs::path(*e);
  return default_cache_root();
}

fs::path sam3_onnx_dir(const Sam3AssetOptions &opts) {
  if (auto e = env("REUSEX_SAM3_ONNX_DIR"))
    return fs::path(*e);
  return resolve_models_dir(opts.models_dir) / kSubDir / "onnx";
}

fs::path sam3_engine_dir(const Sam3AssetOptions &opts) {
#ifdef REUSEX_USE_TENSORRT
  return resolve_models_dir(opts.models_dir) / kSubDir / "engines" /
         engine_cache_key();
#else
  return resolve_models_dir(opts.models_dir) / kSubDir / "engines";
#endif
}

PrepProgress sam3_status(const Sam3AssetOptions &opts) {
  const fs::path onnx_dir = sam3_onnx_dir(opts);
  if (!fs::exists(onnx_dir))
    return {PrepState::absent, 0.0f, "ONNX bundle not present"};
  if (const auto missing = missing_onnx_files(onnx_dir); !missing.empty())
    return {PrepState::absent, 0.0f,
            fmt::format("ONNX bundle incomplete (missing: {})", join(missing))};

  if (!opts.use_cuda)
    return {PrepState::ready, 1.0f, "ONNX present (CPU backend)"};

#ifdef REUSEX_USE_TENSORRT
  const fs::path engine_dir = sam3_engine_dir(opts);
  if (const auto missing = missing_engine_files(onnx_dir, engine_dir);
      !missing.empty())
    return {PrepState::not_built, 0.0f,
            fmt::format("ONNX present, engines not built (missing: {})",
                        join(missing))};
  return {PrepState::ready, 1.0f, "engines built"};
#else
  return {PrepState::error, 0.0f, "TensorRT backend not built"};
#endif
}

fs::path prepare_sam3_model(const Sam3AssetOptions &opts,
                            const ProgressCallback &cb) {
  const fs::path onnx_dir = sam3_onnx_dir(opts);
  throw_if_cancelled(opts.cancel, "start");

  // 1. Ensure the portable ONNX bundle is present and complete.
  if (auto missing = missing_onnx_files(onnx_dir); !missing.empty()) {
    if (env("REUSEX_SAM3_ONNX_DIR"))
      // A user-provided export: never download into (or replace) it.
      throw std::runtime_error(fmt::format(
          "$REUSEX_SAM3_ONNX_DIR={} is not a complete SAM3 ONNX export "
          "(missing: {}). Re-run `make -C python export` or unset the "
          "variable to use the managed model.",
          onnx_dir.string(), join(missing)));
    if (!opts.allow_download)
      throw std::runtime_error(fmt::format(
          "SAM3 model not found at {} (missing: {}) and downloads are "
          "disabled. Provide it via REUSEX_SAM3_ONNX_DIR or `make -C python "
          "export`.",
          onnx_dir.string(), join(missing)));

    const std::string url =
        !opts.manifest_url.empty() ? opts.manifest_url : kDefaultManifestUrl;
    if (url.empty())
      throw std::runtime_error(
          "SAM3 model not present and no download manifest is configured. "
          "Point REUSEX_SAM3_ONNX_DIR at a local ONNX export, or set a "
          "manifest URL once the release bundle is published.");

    std::lock_guard<std::mutex> lock(download_mutex());
    // Another thread (the other provisioning slot) may have finished it
    // while we waited for the lock.
    if (!missing_onnx_files(onnx_dir).empty())
      download_bundle(url, onnx_dir, cb, opts.cancel);
  }

  // 2. CPU/ONNX backend loads the ONNX directly.
  if (!opts.use_cuda) {
    report(cb, PrepState::ready, 1.0f, "ready (ONNX/CPU)");
    return onnx_dir;
  }

  // 3. CUDA: build (or reuse) the device-specific engines.
#ifdef REUSEX_USE_TENSORRT
  const fs::path engine_dir = sam3_engine_dir(opts);
  // A text-only fallback build is loadable (not missing) but is retried
  // here, once per preparation, in case its failure was not the export's.
  if (!missing_engine_files(onnx_dir, engine_dir).empty() ||
      !fallback_engines(engine_dir).empty())
    build_engines(onnx_dir, engine_dir, cb, opts.cancel);
  report(cb, PrepState::ready, 1.0f, "ready (TensorRT engines)");
  return engine_dir;
#else
  throw std::runtime_error(
      "SAM3 CUDA path requested but the TensorRT backend is not built "
      "(configure with -DWITH_CUDA=ON).");
#endif
}

} // namespace reusex::vision::sam3

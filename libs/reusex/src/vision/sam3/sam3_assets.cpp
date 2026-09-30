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

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

namespace reusex::vision::sam3 {

namespace fs = std::filesystem;

// The managed model directory name (SAM 3.1 uses the SAM 3 detector 1:1 plus a
// new tracker; the on-disk layout is shared).
static constexpr const char *kSubDir = "sam3.1";

// TODO: Publish the SAM 3.1 ONNX bundle release and pin its manifest URL here.
// category=Vision estimate=2h
// Description: The portable ONNX + tokenizer.json + tracker-meta.json +
//   engine-build.json are a derivative of Meta's gated facebook/sam3.1
//   checkpoint. The SAM License permits redistributing them ONLY under the SAM
//   License with LICENSE_SAM.txt bundled (see docs/sam3.1-tensorrt.md). Once
//   the GitHub Release asset + manifest.json (files + sha256) exist, set this
//   URL. Until then, point REUSEX_SAM3_ONNX_DIR at a local `make export`
//   output.
static constexpr const char *kDefaultManifestUrl = "";

const char *to_string(PrepState state) {
  switch (state) {
  case PrepState::absent:
    return "absent";
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

/// Download `url` to `dest` atomically. Returns false on a 404 (caller decides
/// whether that is fatal); throws on any other transport/HTTP error.
bool download_to(const std::string &url, const fs::path &dest) {
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
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "reusex-sam3/1.0");

  const CURLcode res = curl_easy_perform(curl);
  out.close();

  if (res != CURLE_OK) {
    std::error_code ec;
    fs::remove(tmp, ec);
    throw std::runtime_error(
        fmt::format("download: {} failed: {}", url, curl_easy_strerror(res)));
  }

  long code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
  if (code == 404) {
    std::error_code ec;
    fs::remove(tmp, ec);
    return false;
  }
  if (code >= 400) {
    std::error_code ec;
    fs::remove(tmp, ec);
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

Manifest fetch_manifest(const std::string &manifest_url) {
  const fs::path tmp = fs::temp_directory_path() /
                       fmt::format("reusex-sam3-manifest-{}.json",
                                   std::hash<std::string>{}(manifest_url));
  if (!download_to(manifest_url, tmp))
    throw std::runtime_error(
        fmt::format("manifest not found (404): {}", manifest_url));

  std::ifstream in(tmp);
  nlohmann::json j;
  in >> j;
  std::error_code ec;
  fs::remove(tmp, ec);

  Manifest m;
  m.base_url = j.value("base_url", std::string{});
  for (const auto &f : j.at("files")) {
    ManifestFile mf;
    mf.name = f.at("name").get<std::string>();
    mf.url = f.value("url", std::string{});
    mf.sha256 = f.value("sha256", std::string{});
    mf.optional = f.value("optional", false);
    m.files.push_back(std::move(mf));
  }
  return m;
}

bool has_onnx(const fs::path &onnx_dir) {
  return fs::exists(onnx_dir / "vision-encoder.onnx") &&
         fs::exists(onnx_dir / "engine-build.json");
}

void download_bundle(const std::string &manifest_url, const fs::path &onnx_dir,
                     const ProgressCallback &cb) {
  report(cb, PrepState::downloading, 0.0f, "fetching manifest");
  const Manifest m = fetch_manifest(manifest_url);
  fs::create_directories(onnx_dir);

  const std::size_t n = m.files.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto &f = m.files[i];
    const fs::path dest = onnx_dir / f.name;
    const std::string url = !f.url.empty() ? f.url : (m.base_url + f.name);

    report(cb, PrepState::downloading, n ? float(i) / float(n) : 0.0f,
           fmt::format("downloading {}", f.name));
    reusex::info("SAM3 assets: downloading {} → {}", url, dest.string());

    const bool got = download_to(url, dest);
    if (!got) {
      if (f.optional) {
        reusex::debug("SAM3 assets: optional file absent, skipping: {}",
                      f.name);
        continue;
      }
      throw std::runtime_error(
          fmt::format("SAM3 assets: required file missing (404): {}", url));
    }
    if (!f.sha256.empty()) {
      const std::string got_hash = sha256_file(dest);
      if (got_hash != f.sha256) {
        std::error_code ec;
        fs::remove(dest, ec);
        throw std::runtime_error(
            fmt::format("SAM3 assets: sha256 mismatch for {}:\n  expected {}\n "
                        " got      {}",
                        f.name, f.sha256, got_hash));
      }
    }
  }
  report(cb, PrepState::downloading, 1.0f, "download complete");
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

/// Copy tokenizer.json / tracker-meta.json alongside the engines so the engine
/// dir is a self-contained model directory create_model_from_path can load.
void copy_metadata(const fs::path &onnx_dir, const fs::path &engine_dir) {
  for (const char *name : {"tokenizer.json", "tracker-meta.json"}) {
    const fs::path src = onnx_dir / name;
    if (fs::exists(src))
      fs::copy_file(src, engine_dir / name,
                    fs::copy_options::overwrite_existing);
  }
}

void build_engines(const fs::path &onnx_dir, const fs::path &engine_dir,
                   const ProgressCallback &cb) {
  const auto profiles =
      EngineBuildProfiles::from_file(onnx_dir / "engine-build.json");
  fs::create_directories(engine_dir);

  // Only build engines whose ONNX is actually present (a detector-only bundle
  // lacks the tracker graphs).
  std::vector<std::string> to_build;
  for (const auto &[name, _] : profiles.engines)
    if (fs::exists(onnx_dir / (name + ".onnx")))
      to_build.push_back(name);

  const std::size_t n = to_build.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::string &name = to_build[i];
    const fs::path engine = engine_dir / (name + ".engine");
    if (fs::exists(engine))
      continue; // already built for this device/TRT
    report(cb, PrepState::building, n ? float(i) / float(n) : 0.0f,
           fmt::format("building {} engine", name));

    tensor_rt::EngineBuildRequest req;
    req.onnx_path = onnx_dir / (name + ".onnx");
    req.engine_path = engine;
    req.profile = *profiles.find(name);
    tensor_rt::build_engine(req);
  }
  copy_metadata(onnx_dir, engine_dir);
  report(cb, PrepState::building, 1.0f, "engine build complete");
}

bool engines_ready(const fs::path &onnx_dir, const fs::path &engine_dir) {
  if (!fs::exists(engine_dir / "vision-encoder.engine"))
    return false;
  // Every buildable engine (onnx present) must have its .engine.
  EngineBuildProfiles profiles;
  try {
    profiles = EngineBuildProfiles::from_file(onnx_dir / "engine-build.json");
  } catch (...) {
    return false;
  }
  for (const auto &[name, _] : profiles.engines)
    if (fs::exists(onnx_dir / (name + ".onnx")) &&
        !fs::exists(engine_dir / (name + ".engine")))
      return false;
  return true;
}
#endif // REUSEX_USE_TENSORRT

} // namespace

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
  if (!has_onnx(onnx_dir))
    return {PrepState::absent, 0.0f, "ONNX bundle not present"};

  if (!opts.use_cuda)
    return {PrepState::ready, 1.0f, "ONNX present (CPU backend)"};

#ifdef REUSEX_USE_TENSORRT
  const fs::path engine_dir = sam3_engine_dir(opts);
  if (engines_ready(onnx_dir, engine_dir))
    return {PrepState::ready, 1.0f, "engines built"};
  return {PrepState::building, 0.0f, "ONNX present, engines not built"};
#else
  return {PrepState::error, 0.0f, "TensorRT backend not built"};
#endif
}

fs::path prepare_sam3_model(const Sam3AssetOptions &opts,
                            const ProgressCallback &cb) {
  const fs::path onnx_dir = sam3_onnx_dir(opts);

  // 1. Ensure the portable ONNX bundle is present.
  if (!has_onnx(onnx_dir)) {
    if (!opts.allow_download)
      throw std::runtime_error(fmt::format(
          "SAM3 model not found at {} and downloads are disabled. Provide it "
          "via REUSEX_SAM3_ONNX_DIR or `make -C python export`.",
          onnx_dir.string()));

    const std::string url =
        !opts.manifest_url.empty() ? opts.manifest_url : kDefaultManifestUrl;
    if (url.empty())
      throw std::runtime_error(
          "SAM3 model not present and no download manifest is configured. "
          "Point REUSEX_SAM3_ONNX_DIR at a local ONNX export, or set a "
          "manifest URL once the release bundle is published.");
    download_bundle(url, onnx_dir, cb);
    if (!has_onnx(onnx_dir))
      throw std::runtime_error(
          fmt::format("SAM3 download completed but {} is incomplete (missing "
                      "vision-encoder.onnx / engine-build.json)",
                      onnx_dir.string()));
  }

  // 2. CPU/ONNX backend loads the ONNX directly.
  if (!opts.use_cuda) {
    report(cb, PrepState::ready, 1.0f, "ready (ONNX/CPU)");
    return onnx_dir;
  }

  // 3. CUDA: build (or reuse) the device-specific engines.
#ifdef REUSEX_USE_TENSORRT
  const fs::path engine_dir = sam3_engine_dir(opts);
  if (!engines_ready(onnx_dir, engine_dir))
    build_engines(onnx_dir, engine_dir, cb);
  report(cb, PrepState::ready, 1.0f, "ready (TensorRT engines)");
  return engine_dir;
#else
  throw std::runtime_error(
      "SAM3 CUDA path requested but the TensorRT backend is not built "
      "(configure with -DWITH_CUDA=ON).");
#endif
}

} // namespace reusex::vision::sam3

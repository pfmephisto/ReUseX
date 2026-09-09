// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Covers reusex::vision::BackendFactory path sniffing (#205):
// detect_backend() extension mapping + directory scanning, and detect_model()
// name/directory heuristics. Both are pure filesystem inspection, so the whole
// file runs on CPU against empty placeholder files — no model weights and no
// GPU (models/ is gitignored, see models/README.md).
//
// UNTESTED HERE, and why:
//   * BackendFactory::create() for tensor_rt / libtorch / onnx_runtime /
//     openvino. Which branch those take is decided by the REUSEX_USE_*
//     compile definitions, and configure_ml_backends() attaches those PRIVATE
//     to the reusex_vision target — they are invisible in this TU, so
//     asserting here would exercise a different branch than the library's.
//     Constructing a real backend also pulls in the TensorRT/ONNX runtime.
//     Only the unconditional opencv / dnn / unknown throws are asserted.
//   * IMLBackend::create_model() / create_video_model() / create_dataset() and
//     everything downstream (Yolo, Sam3, Sam3p1 inference, engine
//     deserialization, CUDA kernels) — all require weights and a GPU.

#include <catch2/catch_test_macros.hpp>

#include <vision/BackendFactory.hpp>

#include "../../support/temp_path.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

using reusex::vision::Backend;
using reusex::vision::BackendFactory;
using reusex::vision::Model;

namespace fs = std::filesystem;

namespace {

struct TempDir : reusex::test_support::TempDir {
  TempDir() : reusex::test_support::TempDir("test_backend_factory") {}
};

/// Create an empty placeholder file. Detection only ever looks at the path, so
/// the contents are irrelevant and deliberately absent.
fs::path touch(const fs::path &dir, const std::string &name) {
  const auto p = dir / name;
  fs::create_directories(p.parent_path());
  std::ofstream out(p);
  out.close();
  return p;
}

} // namespace

// ── detect_backend: extension mapping ─────────────────────────────────────

TEST_CASE("DetectBackend_EngineExtension_ReturnsTensorRt",
          "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "model.engine")) ==
          Backend::tensor_rt);
}

TEST_CASE("DetectBackend_PyTorchExtensions_ReturnsLibTorch",
          "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "yolo11l.pt")) ==
          Backend::libtorch);
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "weights.pth")) ==
          Backend::libtorch);
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "m.torchscript")) ==
          Backend::libtorch);
}

TEST_CASE("DetectBackend_OnnxAndOpenVinoExtensions_ReturnsMatchingBackend",
          "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "foo.onnx")) ==
          Backend::onnx_runtime);
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "net.xml")) ==
          Backend::openvino);
  REQUIRE(BackendFactory::detect_backend(touch(dir.path, "net.bin")) ==
          Backend::openvino);
}

TEST_CASE("DetectBackend_UnknownOrMissingExtension_ReturnsUnknown",
          "[vision][backend]") {
  TempDir dir;

  SECTION("unrecognized extension") {
    REQUIRE(BackendFactory::detect_backend(touch(dir.path, "notes.txt")) ==
            Backend::unknown);
  }
  SECTION("no extension at all") {
    REQUIRE(BackendFactory::detect_backend(touch(dir.path, "model")) ==
            Backend::unknown);
  }
  SECTION("a dotfile is an extension-less name, not an extension") {
    // path(".engine").extension() is empty — the whole name is the stem — so
    // this must NOT be taken for a TensorRT engine.
    REQUIRE(BackendFactory::detect_backend(touch(dir.path, ".engine")) ==
            Backend::unknown);
  }
}

TEST_CASE("DetectBackend_PathDoesNotExist_ReturnsUnknown",
          "[vision][backend]") {
  TempDir dir;
  // Neither is_regular_file nor is_directory holds, so nothing is inspected —
  // the extension is never consulted.
  REQUIRE(BackendFactory::detect_backend(dir.path / "absent.engine") ==
          Backend::unknown);
}

// ── detect_backend: directory scanning ────────────────────────────────────

TEST_CASE("DetectBackend_DirectoryWithRecognizableFile_ReturnsMatchingBackend",
          "[vision][backend]") {
  TempDir dir;
  touch(dir.path, "README.md");
  touch(dir.path, "vision-encoder.onnx");
  REQUIRE(BackendFactory::detect_backend(dir.path) == Backend::onnx_runtime);
}

TEST_CASE("DetectBackend_EmptyDirectory_ReturnsUnknown", "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_backend(dir.path) == Backend::unknown);
}

TEST_CASE("DetectBackend_DirectoryWithNoModelFile_ReturnsUnknown",
          "[vision][backend]") {
  TempDir dir;
  touch(dir.path, "README.md");
  touch(dir.path, "tokenizer.json");
  REQUIRE(BackendFactory::detect_backend(dir.path) == Backend::unknown);
}

TEST_CASE("DetectBackend_NestedEngineFile_IgnoresSubdirectories",
          "[vision][backend]") {
  TempDir dir;
  // Only entries that are regular files are considered, so an engine buried
  // one level down must not be found.
  touch(dir.path, "nested/model.engine");
  REQUIRE(BackendFactory::detect_backend(dir.path) == Backend::unknown);
}

// ── detect_model: name shortcuts ──────────────────────────────────────────

TEST_CASE("DetectModel_PtExtension_DefaultsToYolo", "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "yolo11l.pt")) ==
          Model::yolo);
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "yolo11l-seg.pt")) ==
          Model::yolo);
  // A path that does not exist still classifies by name.
  REQUIRE(BackendFactory::detect_model(dir.path / "absent.pt") == Model::yolo);
}

TEST_CASE("DetectModel_Sam3OrSam2Name_ReturnsSam3", "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "sam3_x.engine")) ==
          Model::sam3);
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "sam2_hiera.pt")) ==
          Model::sam3);
}

TEST_CASE("DetectModel_MixedCaseName_ReturnsMatchingModel",
          "[vision][backend]") {
  TempDir dir;
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "SAM3_Large.engine")) ==
          Model::sam3);
  REQUIRE(BackendFactory::detect_model(touch(dir.path, "SAM3.1-video")) ==
          Model::sam3p1);
}

TEST_CASE("DetectModel_Sam3p1Name_PrefersOverPlainSam3", "[vision][backend]") {
  TempDir dir;

  SECTION("dotted spelling") {
    // Regression guard: filename() must be used, not stem(). stem() of
    // "sam3.1-video" is "sam3", which would mis-route to the plain SAM3 path
    // and silently skip the video tracker.
    fs::create_directories(dir.path / "sam3.1-video");
    REQUIRE(BackendFactory::detect_model(dir.path / "sam3.1-video") ==
            Model::sam3p1);
  }
  SECTION("p-spelling") {
    REQUIRE(BackendFactory::detect_model(touch(dir.path, "sam3p1.engine")) ==
            Model::sam3p1);
  }
}

// ── detect_model: directory sniffing ──────────────────────────────────────

TEST_CASE("DetectModel_VisionEncoderDirectory_ReturnsSam3",
          "[vision][backend]") {
  TempDir dir;
  const auto models = dir.path / "models"; // neutral name, no sam3 shortcut
  touch(models, "vision-encoder.onnx");
  touch(models, "prompt-encoder.onnx");
  REQUIRE(BackendFactory::detect_model(models) == Model::sam3);
}

TEST_CASE("DetectModel_TrackerFileSet_ReturnsSam3p1OnlyWhenComplete",
          "[vision][backend]") {
  TempDir dir;
  const auto models = dir.path / "models";
  touch(models, "vision-encoder.engine");
  touch(models, "tracker-memory-encoder.engine");

  SECTION("memory-attention missing -> plain sam3") {
    REQUIRE(BackendFactory::detect_model(models) == Model::sam3);
  }
  SECTION("all three present -> sam3p1") {
    touch(models, "tracker-memory-attention.engine");
    REQUIRE(BackendFactory::detect_model(models) == Model::sam3p1);
  }
}

TEST_CASE("DetectModel_UnrecognizedDirectory_FallsBackToYolo",
          "[vision][backend]") {
  TempDir dir;
  const auto models = dir.path / "models";
  touch(models, "weights.engine");
  touch(models, "labels.txt");
  REQUIRE(BackendFactory::detect_model(models) == Model::yolo);
}

TEST_CASE("DetectModel_EncoderNamedFile_DoesNotTriggerDirectorySniffing",
          "[vision][backend]") {
  TempDir dir;
  // A regular file named after an encoder is not a model directory; the
  // directory_iterator branch is skipped and the YOLO default applies.
  REQUIRE(BackendFactory::detect_model(
              touch(dir.path, "vision-encoder-notes.txt")) == Model::yolo);
}

// ── create(): the unconditional error paths ───────────────────────────────

TEST_CASE("Create_UnimplementedBackends_Throws", "[vision][backend]") {
  // opencv, dnn and unknown throw in every build configuration — unlike the
  // REUSEX_USE_*-gated backends (see the untested list at the top).
  REQUIRE_THROWS_AS(BackendFactory::create(Backend::opencv),
                    std::runtime_error);
  REQUIRE_THROWS_AS(BackendFactory::create(Backend::dnn), std::runtime_error);
  REQUIRE_THROWS_AS(BackendFactory::create(Backend::unknown),
                    std::runtime_error);
}

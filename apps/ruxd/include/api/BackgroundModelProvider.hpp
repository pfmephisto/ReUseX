// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The provisioning state machine behind IModelProvider, independent of what
// is being provisioned. ruxd's main plugs in the SAM3 prepare/probe
// functions (reusex::vision::sam3); keeping the vision closure out of here lets
// the state machine live in ruxd_api_lib and be unit-tested in the light test
// binary with fake prepare functions.

#pragma once

#include "api/ModelProvider.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace ruxd::api {

/// Runs model preparation on a background thread per `use_cuda` slot.
///
/// * ensure() starts preparation when nothing has run yet, and retries after a
///   failure once @p retry_backoff has elapsed — an error is never sticky
///   until restart.
/// * status() reports the in-flight / finished state of a slot, or the pure
///   disk probe when that slot has never run.
/// * The destructor raises the stop flag handed to @p prepare and joins the
///   workers. Preparation is expected to poll that flag (between download
///   chunks and between engine builds); a single uninterruptible step — one
///   TensorRT engine build — bounds how long shutdown can take.
class BackgroundModelProvider : public IModelProvider {
    public:
  /// Progress sink handed to PrepareFn; `model_path` is ignored.
  using ProgressFn = std::function<void(const ModelPrepStatus &)>;
  /// Blocking preparation. Returns the loadable model directory; throws on
  /// failure. Must return promptly (by throwing) once @p stop reads true.
  using PrepareFn =
      std::function<std::string(bool use_cuda, const ProgressFn &progress,
                                const std::atomic<bool> &stop)>;
  /// Side-effect-free probe of what is already on disk.
  using ProbeFn = std::function<ModelPrepStatus(bool use_cuda)>;

  /// @param explicit_model  when non-empty, every call reports it as ready and
  ///                        nothing is ever prepared (`--sam3-model`).
  BackgroundModelProvider(
      PrepareFn prepare, ProbeFn probe, std::string explicit_model = {},
      std::chrono::milliseconds retry_backoff = std::chrono::seconds(10));
  ~BackgroundModelProvider() override;

  BackgroundModelProvider(const BackgroundModelProvider &) = delete;
  BackgroundModelProvider &operator=(const BackgroundModelProvider &) = delete;

  ModelPrepStatus ensure(bool use_cuda) override;
  ModelPrepStatus status(bool use_cuda) override;

    private:
  struct Slot;
  void run(Slot &slot, bool use_cuda);
  void start(Slot &slot, bool use_cuda);

  PrepareFn prepare_;
  ProbeFn probe_;
  std::string explicit_model_;
  std::chrono::milliseconds retry_backoff_;
  std::atomic<bool> stop_{false};
  std::unique_ptr<Slot> slots_[2]; // [0] = CPU/ONNX, [1] = CUDA/TensorRT
};

} // namespace ruxd::api

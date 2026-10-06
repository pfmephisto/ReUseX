// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/BackgroundModelProvider.hpp"

#include <spdlog/spdlog.h>

#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace rux::gui {

struct BackgroundModelProvider::Slot {
  std::mutex m;
  std::thread worker;
  bool started = false; ///< a worker has been launched at least once
  bool running = false; ///< the current worker has not finished yet
  std::chrono::steady_clock::time_point failed_at{};
  ModelPrepStatus status;
};

BackgroundModelProvider::BackgroundModelProvider(
    PrepareFn prepare, ProbeFn probe, std::string explicit_model,
    std::chrono::milliseconds retry_backoff)
    : prepare_(std::move(prepare)), probe_(std::move(probe)),
      explicit_model_(std::move(explicit_model)),
      retry_backoff_(retry_backoff) {
  for (auto &slot : slots_)
    slot = std::make_unique<Slot>();
}

BackgroundModelProvider::~BackgroundModelProvider() {
  // Cooperative cancellation: without it the join below would wait out a
  // multi-minute download or engine build.
  stop_.store(true);
  for (auto &slot : slots_) {
    std::thread worker;
    {
      std::lock_guard<std::mutex> lock(slot->m);
      worker = std::move(slot->worker);
    }
    if (worker.joinable())
      worker.join();
  }
}

void BackgroundModelProvider::start(Slot &slot, bool use_cuda) {
  // Caller holds slot.m. A previous worker, if any, has already finished
  // (running == false), so this join returns immediately.
  if (slot.worker.joinable())
    slot.worker.join();
  slot.started = true;
  slot.running = true;
  slot.status = {"downloading", 0.0f, "preparing managed SAM3 model", "", {}};
  slot.worker = std::thread([this, &slot, use_cuda] { run(slot, use_cuda); });
}

ModelPrepStatus BackgroundModelProvider::ensure(bool use_cuda) {
  if (!explicit_model_.empty())
    return {"ready", 1.0f, "explicit model", explicit_model_, {}};

  Slot &slot = *slots_[use_cuda ? 1 : 0];
  std::lock_guard<std::mutex> lock(slot.m);
  if (stop_.load())
    return slot.status;
  if (!slot.started) {
    start(slot, use_cuda);
  } else if (!slot.running && slot.status.state == "error" &&
             std::chrono::steady_clock::now() - slot.failed_at >=
                 retry_backoff_) {
    spdlog::info("Retrying managed SAM3 model preparation (cuda={}) after: {}",
                 use_cuda, slot.status.message);
    start(slot, use_cuda);
  }
  return slot.status;
}

ModelPrepStatus BackgroundModelProvider::status(bool use_cuda) {
  if (!explicit_model_.empty())
    return {"ready", 1.0f, "explicit model", explicit_model_, {}};

  Slot &slot = *slots_[use_cuda ? 1 : 0];
  {
    std::lock_guard<std::mutex> lock(slot.m);
    if (slot.started)
      return slot.status;
  }
  // Never run in this process: pure probe of what is already on disk.
  return probe_(use_cuda);
}

void BackgroundModelProvider::run(Slot &slot, bool use_cuda) {
  ModelPrepStatus final_status;
  try {
    const ProgressFn progress = [&slot](const ModelPrepStatus &p) {
      std::lock_guard<std::mutex> lock(slot.m);
      slot.status.state = p.state;
      slot.status.progress = p.progress;
      slot.status.message = p.message;
    };
    std::string dir = prepare_(use_cuda, progress, stop_);
    if (dir.empty())
      throw std::runtime_error("model preparation returned no directory");
    final_status = {"ready", 1.0f, "ready", std::move(dir), {}};
  } catch (const std::exception &e) {
    if (stop_.load())
      spdlog::info("Managed SAM3 model preparation stopped: {}", e.what());
    else
      spdlog::error("Managed SAM3 model preparation failed (cuda={}): {}",
                    use_cuda, e.what());
    final_status = {"error", 0.0f, e.what(), "", {}};
  } catch (...) {
    spdlog::error("Managed SAM3 model preparation failed (cuda={}): unknown "
                  "exception",
                  use_cuda);
    final_status = {"error", 0.0f, "unknown error", "", {}};
  }

  std::lock_guard<std::mutex> lock(slot.m);
  slot.status = std::move(final_status);
  if (slot.status.state == "error") {
    slot.failed_at = std::chrono::steady_clock::now();
    slot.status.message +=
        " (will retry on the next request after " +
        std::to_string(
            std::chrono::duration_cast<std::chrono::seconds>(retry_backoff_)
                .count()) +
        " s)";
  }
  slot.running = false;
}

} // namespace rux::gui

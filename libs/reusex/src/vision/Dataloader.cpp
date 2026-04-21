// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "core/logging.hpp"
#include "vision/Dataloader.hpp"
#include "vision/IData.hpp"
#include "vision/IDataset.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <map>
#include <random>
#include <span>
#include <thread>
#include <vector>

namespace ReUseX::vision {

Dataloader::Dataloader(IDataset &dataset, size_t batch_size, bool shuffle,
                       size_t num_workers, size_t prefetch_batches)
    : dataset_(dataset), batch_size_(batch_size), shuffle_(shuffle),
      num_workers_(num_workers), prefetch_batches_(prefetch_batches),
      dataset_size_(dataset.size()),
      num_batches_((dataset_size_ + batch_size - 1) / batch_size),
      stop_workers_(false), epoch_finished_(false) {
  ReUseX::core::info(
      "Initializing Dataloader: dataset_size={}, batch_size={}, "
      "num_batches={}, shuffle={}, num_workers={}, prefetch_batches={}",
      dataset_size_, batch_size_, num_batches_, shuffle_, num_workers_,
      prefetch_batches_);
  indices_.reserve(dataset_size_);
  for (size_t i = 0; i < dataset_size_; ++i) {
    indices_.push_back(i);
  }
}

Dataloader::~Dataloader() {
  ReUseX::core::debug("Destroying Dataloader");
  stop();
}

Dataloader::Iterator::Iterator(Dataloader *loader, size_t batch_idx)
    : loader_(loader), batch_idx_(batch_idx) {}

Dataloader::BatchView Dataloader::Iterator::operator*() const {
  if (!current_batch_) {
    current_batch_ = loader_->get_batch(batch_idx_);
  }
  return BatchView(*current_batch_);
}

Dataloader::Batch &&Dataloader::Iterator::move_batch() {
  if (!current_batch_) {
    current_batch_ = loader_->get_batch(batch_idx_);
  }
  return std::move(*current_batch_);
}

Dataloader::Iterator &Dataloader::Iterator::operator++() {
  ++batch_idx_;
  current_batch_.reset();
  return *this;
}

bool Dataloader::Iterator::operator==(const Iterator &other) const {
  return batch_idx_ == other.batch_idx_;
}

bool Dataloader::Iterator::operator!=(const Iterator &other) const {
  return !(*this == other);
}

Dataloader::Iterator Dataloader::begin() {
  start_epoch();
  return Iterator(this, 0);
}

Dataloader::Iterator Dataloader::end() { return Iterator(this, num_batches_); }

size_t Dataloader::size() const { return num_batches_; }

void Dataloader::set_num_workers(size_t num_workers) {
  ReUseX::core::info("Setting num_workers to {}", num_workers);
  stop();
  num_workers_ = num_workers;
}

void Dataloader::set_prefetch_batches(size_t prefetch_batches) {
  ReUseX::core::info("Setting prefetch_batches to {}", prefetch_batches);
  stop();
  prefetch_batches_ = prefetch_batches;
}

size_t Dataloader::get_num_workers() const { return num_workers_; }
size_t Dataloader::get_prefetch_batches() const { return prefetch_batches_; }

void Dataloader::start_epoch() {
  ReUseX::core::info("Starting new epoch with {} workers", num_workers_);
  stop();

  if (shuffle_) {
    ReUseX::core::debug("Shuffling dataset indices");
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices_.begin(), indices_.end(), g);
  }

  stop_workers_ = false;
  epoch_finished_ = false;
  current_batch_idx_ = 0;

  for (size_t i = 0; i < num_workers_; ++i) {
    workers_.emplace_back(&Dataloader::worker_thread, this);
  }
  ReUseX::core::debug("Started {} worker threads", num_workers_);
}

void Dataloader::stop() {
  ReUseX::core::debug("Stopping dataloader workers");
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_workers_ = true;
    epoch_finished_ = true;
  }
  queue_cv_.notify_all();
  ready_cv_.notify_all();

  for (auto &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  workers_.clear();

  batch_queue_.clear();
  ReUseX::core::debug("All workers stopped");
}

void Dataloader::worker_thread() {
  while (true) {
    size_t batch_idx;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      if (stop_workers_) {
        return;
      }

      queue_cv_.wait(lock, [this] {
        return stop_workers_ || (batch_queue_.size() < prefetch_batches_ &&
                                 current_batch_idx_ < num_batches_);
      });

      if (stop_workers_) {
        return;
      }

      if (current_batch_idx_ >= num_batches_) {
        continue;
      }

      batch_idx = current_batch_idx_++;
    }

    auto batch = load_batch(batch_idx);

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (!stop_workers_) {
        batch_queue_.emplace(batch_idx, std::move(batch));
        ready_cv_.notify_one();
      }
    }
  }
}

Dataloader::Batch Dataloader::load_batch(size_t batch_idx) {
  ReUseX::core::trace(
      "Loading batch {} (indices {} to {})", batch_idx, batch_idx * batch_size_,
      std::min((batch_idx + 1) * batch_size_, dataset_size_) - 1);
  Batch batch;
  size_t start_idx = batch_idx * batch_size_;
  size_t end_idx = std::min(start_idx + batch_size_, dataset_size_);

  batch.reserve(end_idx - start_idx);
  for (size_t i = start_idx; i < end_idx; ++i) {
    batch.push_back(dataset_.get(indices_[i]));
  }

  return batch;
}

std::optional<Dataloader::Batch> Dataloader::get_batch(size_t batch_idx) {
  ReUseX::core::trace("Requesting batch {}", batch_idx);
  std::unique_lock<std::mutex> lock(queue_mutex_);

  ready_cv_.wait(lock, [this, batch_idx] {
    return epoch_finished_ || batch_queue_.count(batch_idx);
  });

  auto it = batch_queue_.find(batch_idx);
  if (it != batch_queue_.end()) {
    auto batch = std::move(it->second);
    batch_queue_.erase(it);
    queue_cv_.notify_one();
    ReUseX::core::trace("Batch {} retrieved from queue", batch_idx);
    return batch;
  }

  if (epoch_finished_) {
    ReUseX::core::debug("Epoch finished, no more batches available");
  }
  return std::nullopt;
}

} // namespace ReUseX::vision

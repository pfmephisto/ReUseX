// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <span>
#include <thread>
#include <vector>

namespace ReUseX::vision {

class Dataloader {
    public:
  using Batch = std::vector<std::unique_ptr<IData>>;
  using BatchView = std::span<std::unique_ptr<IData>>;

  Dataloader(IDataset &dataset, size_t batch_size, bool shuffle = false,
             size_t num_workers = 4, size_t prefetch_batches = 2);

  ~Dataloader();

  class Iterator {
      public:
    using iterator_category = std::input_iterator_tag;
    using value_type = Batch;
    using difference_type = std::ptrdiff_t;
    using pointer = Batch *;
    using reference = Batch &;

    Iterator(Dataloader *loader, size_t batch_idx);

    Iterator(const Iterator &other)
        : loader_(other.loader_), batch_idx_(other.batch_idx_),
          current_batch_(std::nullopt) {}
    Iterator &operator=(const Iterator &other) {
      if (this != &other) {
        loader_ = other.loader_;
        batch_idx_ = other.batch_idx_;
        current_batch_ = std::nullopt;
      }
      return *this;
    }
    Iterator(Iterator &&) = default;
    Iterator &operator=(Iterator &&) = default;

    BatchView operator*() const;

    Batch &&move_batch();

    Iterator &operator++();

    Iterator operator++(int) = delete;

    bool operator==(const Iterator &other) const;

    bool operator!=(const Iterator &other) const;

      private:
    Dataloader *loader_;
    size_t batch_idx_;
    mutable std::optional<Batch> current_batch_;
  };

  Iterator begin();

  Iterator end();

  size_t size() const;

  void set_num_workers(size_t num_workers);

  void set_prefetch_batches(size_t prefetch_batches);

  size_t get_num_workers() const;
  size_t get_prefetch_batches() const;

    private:
  void start_epoch();

  void stop();

  void worker_thread();

  Batch load_batch(size_t batch_idx);

  std::optional<Batch> get_batch(size_t batch_idx);

  IDataset &dataset_;
  size_t batch_size_;
  bool shuffle_;
  size_t num_workers_;
  size_t prefetch_batches_;
  size_t dataset_size_;
  size_t num_batches_;

  std::vector<size_t> indices_;
  std::vector<std::thread> workers_;
  std::queue<std::pair<size_t, Batch>> batch_queue_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable ready_cv_;

  std::atomic<bool> stop_workers_;
  std::atomic<bool> epoch_finished_;
  size_t current_batch_idx_;
};

} // namespace ReUseX::vision

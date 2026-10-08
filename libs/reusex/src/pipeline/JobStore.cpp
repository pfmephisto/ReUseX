// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/JobStore.hpp"

#include <map>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace reusex::pipeline {

class InMemoryJobStore::Impl {
    public:
  explicit Impl(std::size_t max_terminal) : max_terminal_(max_terminal) {}

  void save(std::string_view queue, const JobRecord &record) {
    std::lock_guard<std::mutex> lock(mutex_);
    Queue &q = queues_[std::string(queue)];
    auto [it, inserted] = q.records.insert_or_assign(record.id, record);
    (void)it;
    if (inserted)
      q.order.push_back(record.id);
    if (is_terminal(record.status))
      evict_terminal(q);
  }

  std::optional<JobRecord> find(std::string_view queue,
                                std::string_view id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto q = queues_.find(std::string(queue));
    if (q == queues_.end())
      return std::nullopt;
    auto it = q->second.records.find(std::string(id));
    if (it == q->second.records.end())
      return std::nullopt;
    return it->second;
  }

  std::vector<JobRecord> list(std::string_view queue) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<JobRecord> out;
    auto q = queues_.find(std::string(queue));
    if (q == queues_.end())
      return out;
    out.reserve(q->second.order.size());
    for (auto it = q->second.order.rbegin(); it != q->second.order.rend();
         ++it) {
      auto found = q->second.records.find(*it);
      if (found != q->second.records.end())
        out.push_back(found->second);
    }
    return out;
  }

  void forget(std::string_view queue) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = queues_.find(queue); it != queues_.end())
      queues_.erase(it);
  }

    private:
  struct Queue {
    std::vector<std::string> order; ///< Submission order.
    std::unordered_map<std::string, JobRecord> records;
  };

  /// Drop the oldest terminal records until at most max_terminal_ remain.
  ///
  /// Ordering comes from `order` (submission order), never from the unordered
  /// record map, whose iteration order is not reproducible (STANDARDS §6).
  /// Queued and running records are never candidates: a store that silently
  /// forgot a submitted job would be a worse failure than the memory it bounds.
  void evict_terminal(Queue &q) const {
    std::size_t terminal = 0;
    for (const auto &id : q.order) {
      auto it = q.records.find(id);
      if (it != q.records.end() && is_terminal(it->second.status))
        ++terminal;
    }
    if (terminal <= max_terminal_)
      return;

    std::size_t to_drop = terminal - max_terminal_;
    std::vector<std::string> retained;
    retained.reserve(q.order.size() - to_drop);
    for (const auto &id : q.order) {
      auto it = q.records.find(id);
      if (it == q.records.end())
        continue;
      if (to_drop > 0 && is_terminal(it->second.status)) {
        q.records.erase(it);
        --to_drop;
        continue;
      }
      retained.push_back(id);
    }
    q.order.swap(retained);
  }

  std::size_t max_terminal_;
  mutable std::mutex mutex_;
  std::map<std::string, Queue, std::less<>> queues_;
};

InMemoryJobStore::InMemoryJobStore(std::size_t max_terminal_jobs)
    : impl_(std::make_unique<Impl>(max_terminal_jobs)) {}

InMemoryJobStore::~InMemoryJobStore() = default;

void InMemoryJobStore::save(std::string_view queue, const JobRecord &record) {
  impl_->save(queue, record);
}

std::optional<JobRecord> InMemoryJobStore::find(std::string_view queue,
                                                std::string_view id) const {
  return impl_->find(queue, id);
}

std::vector<JobRecord> InMemoryJobStore::list(std::string_view queue) const {
  return impl_->list(queue);
}

void InMemoryJobStore::forget(std::string_view queue) { impl_->forget(queue); }

} // namespace reusex::pipeline

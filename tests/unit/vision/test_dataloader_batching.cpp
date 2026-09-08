// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Covers reusex::vision::Dataloader batching and visit ordering (#205),
// complementing test_dataloader.cpp which pins the shuffled_indices() RNG
// contract from #219. This file drives the loader itself: batch boundaries,
// what order samples come back in, and that the order is reproducible from a
// seed across constructions, epochs and worker counts (docs/STANDARDS.md §6).
//
// The dataset is a stub over an in-memory ProjectDB whose get() returns the
// index it was handed, so a whole epoch collapses to the vector of indices the
// loader actually visited. No images are decoded, no model is loaded, nothing
// touches a GPU.
//
// UNTESTED HERE, and why:
//   * The concrete datasets (TensorRTDataset / LibTorchDataset /
//     ONNXSam3Dataset) and their image decode + preprocessing — those need
//     the ML runtimes; IDataset's own index/resume logic is covered by
//     test_annotate_resume.cpp.
//   * Prefetch *timing* / throughput. prefetch_batches and num_workers are
//     exercised for correctness, but any assertion about when a batch becomes
//     available would be a race, not a test.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <vision/Dataloader.hpp>
#include <vision/IData.hpp>
#include <vision/IDataset.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::vision::Dataloader;

namespace {

/// A data sample carrying nothing but the index it was built from.
struct IndexData : reusex::vision::IData {
  explicit IndexData(std::size_t i) : index(i) {}
  std::size_t index;
};

/// Minimal IDataset stub. size() comes from the sensor frames in the backing
/// project; get() is a pure function of its argument, so it is safe to call
/// from several worker threads at once (it never touches the sqlite handle).
class IndexDataset : public reusex::vision::IDataset {
    public:
  explicit IndexDataset(std::shared_ptr<ProjectDB> db)
      : IDataset(std::move(db)) {}

  Pair get(const std::size_t index) const override {
    return {std::make_unique<IndexData>(index), index};
  }

  bool save(const std::span<Pair> &) override { return true; }
};

/// An in-memory project holding @p count sensor frames, so IDataset::size()
/// reports @p count.
std::shared_ptr<ProjectDB> make_db(int count) {
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(2, 2, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < count; ++i)
    db->save_sensor_frame(i + 1, color);
  return db;
}

/// The indices one full epoch visited, in consumption order.
std::vector<std::size_t> visit_order(Dataloader &loader) {
  std::vector<std::size_t> order;
  for (auto batch : loader)
    for (const auto &pair : batch)
      order.push_back(pair.second);
  return order;
}

/// The size of each batch one full epoch produced, in order.
std::vector<std::size_t> batch_sizes(Dataloader &loader) {
  std::vector<std::size_t> sizes;
  for (auto batch : loader)
    sizes.push_back(batch.size());
  return sizes;
}

std::vector<std::size_t> iota_vector(std::size_t n) {
  std::vector<std::size_t> v(n);
  std::iota(v.begin(), v.end(), std::size_t{0});
  return v;
}

} // namespace

// ── batch boundaries ──────────────────────────────────────────────────────

TEST_CASE("Dataloader batch count rounds up on a non-divisible dataset",
          "[vision][dataloader]") {
  auto db = make_db(10);
  IndexDataset ds(db);
  REQUIRE(ds.size() == 10);

  Dataloader loader(ds, /*batch_size=*/4, /*shuffle=*/false, /*num_workers=*/2);

  REQUIRE(loader.size() == 3); // ceil(10 / 4)
  REQUIRE(batch_sizes(loader) == std::vector<std::size_t>{4, 4, 2});
}

TEST_CASE("Dataloader emits equal batches when the size divides exactly",
          "[vision][dataloader]") {
  auto db = make_db(12);
  IndexDataset ds(db);

  Dataloader loader(ds, 4, false, 2);

  REQUIRE(loader.size() == 3);
  REQUIRE(batch_sizes(loader) == std::vector<std::size_t>{4, 4, 4});
}

TEST_CASE("Dataloader handles a batch larger than the dataset",
          "[vision][dataloader]") {
  auto db = make_db(3);
  IndexDataset ds(db);

  Dataloader loader(ds, /*batch_size=*/16, false, 2);

  REQUIRE(loader.size() == 1);
  REQUIRE(batch_sizes(loader) == std::vector<std::size_t>{3});
  REQUIRE(visit_order(loader) == iota_vector(3));
}

TEST_CASE("Dataloader with batch size one yields one sample per batch",
          "[vision][dataloader]") {
  auto db = make_db(5);
  IndexDataset ds(db);

  Dataloader loader(ds, 1, false, 2);

  REQUIRE(loader.size() == 5);
  REQUIRE(batch_sizes(loader) == std::vector<std::size_t>(5, 1));
  REQUIRE(visit_order(loader) == iota_vector(5));
}

TEST_CASE("Dataloader over an empty dataset produces no batches",
          "[vision][dataloader]") {
  auto db = make_db(0);
  IndexDataset ds(db);
  REQUIRE(ds.size() == 0);

  Dataloader loader(ds, 4, false, 2);

  REQUIRE(loader.size() == 0);
  REQUIRE(loader.begin() == loader.end());
  REQUIRE(visit_order(loader).empty());
}

// ── ordering ──────────────────────────────────────────────────────────────

TEST_CASE("Dataloader without shuffle visits the dataset in order",
          "[vision][dataloader]") {
  auto db = make_db(10);
  IndexDataset ds(db);

  Dataloader loader(ds, 3, /*shuffle=*/false, 2);

  REQUIRE(visit_order(loader) == iota_vector(10));
}

TEST_CASE("Dataloader shuffle visits every sample exactly once",
          "[vision][dataloader]") {
  auto db = make_db(37); // deliberately not a multiple of the batch size
  IndexDataset ds(db);

  Dataloader loader(ds, 8, /*shuffle=*/true, 3, 2, Dataloader::default_seed);

  auto order = visit_order(loader);
  REQUIRE(order.size() == 37);

  auto sorted = order;
  std::sort(sorted.begin(), sorted.end());
  REQUIRE(sorted == iota_vector(37));
  REQUIRE(order != sorted); // and it really was shuffled
}

TEST_CASE("Dataloader visit order matches shuffled_indices for the same seed",
          "[vision][dataloader]") {
  constexpr std::uint32_t kSeed = 1234;
  auto db = make_db(20);
  IndexDataset ds(db);

  Dataloader loader(ds, 6, true, 2, 2, std::optional<std::uint32_t>(kSeed));

  // shuffled_indices() is documented as the single source of truth for
  // ordering; the loader must not add a permutation of its own on top.
  REQUIRE(visit_order(loader) == Dataloader::shuffled_indices(
                                     20, std::optional<std::uint32_t>(kSeed)));
}

// ── determinism (STANDARDS §6) ────────────────────────────────────────────

TEST_CASE("Two Dataloaders with the same seed visit in identical order",
          "[vision][dataloader]") {
  auto db_a = make_db(64);
  auto db_b = make_db(64);
  IndexDataset ds_a(db_a);
  IndexDataset ds_b(db_b);

  Dataloader a(ds_a, 7, true, 2, 2, std::optional<std::uint32_t>(99));
  Dataloader b(ds_b, 7, true, 2, 2, std::optional<std::uint32_t>(99));

  REQUIRE(visit_order(a) == visit_order(b));
}

TEST_CASE("Dataloaders with different seeds visit in different order",
          "[vision][dataloader]") {
  // 64! possible orders, so a chance collision is not a credible flake.
  auto db_a = make_db(64);
  auto db_b = make_db(64);
  IndexDataset ds_a(db_a);
  IndexDataset ds_b(db_b);

  Dataloader a(ds_a, 7, true, 2, 2, std::optional<std::uint32_t>(1));
  Dataloader b(ds_b, 7, true, 2, 2, std::optional<std::uint32_t>(2));

  REQUIRE(visit_order(a) != visit_order(b));
}

TEST_CASE("A seeded Dataloader repeats its order across epochs",
          "[vision][dataloader]") {
  // Dataloader.hpp documents a held seed as "repeatable across epochs and
  // runs" — re-iterating restarts the epoch and must reshuffle identically.
  auto db = make_db(32);
  IndexDataset ds(db);

  Dataloader loader(ds, 5, true, 2, 2, Dataloader::default_seed);

  const auto first = visit_order(loader);
  const auto second = visit_order(loader);

  REQUIRE(first.size() == 32);
  REQUIRE(first == second);
}

TEST_CASE("Dataloader order is independent of the worker count",
          "[vision][dataloader]") {
  // Batches are produced concurrently but consumed by index, so the number of
  // workers must not perturb what the caller sees.
  auto db_1 = make_db(50);
  auto db_8 = make_db(50);
  IndexDataset ds_1(db_1);
  IndexDataset ds_8(db_8);

  Dataloader one(ds_1, 6, true, /*num_workers=*/1, 2, Dataloader::default_seed);
  Dataloader eight(ds_8, 6, true, /*num_workers=*/8, 4,
                   Dataloader::default_seed);

  REQUIRE(visit_order(one) == visit_order(eight));
}

TEST_CASE("Dataloader order survives changing the worker count mid-life",
          "[vision][dataloader]") {
  // set_num_workers() stops the current epoch; the next one must still be the
  // same seeded permutation.
  auto db = make_db(24);
  IndexDataset ds(db);

  Dataloader loader(ds, 5, true, 2, 2, Dataloader::default_seed);
  const auto before = visit_order(loader);

  loader.set_num_workers(4);
  loader.set_prefetch_batches(1);
  REQUIRE(loader.get_num_workers() == 4);
  REQUIRE(loader.get_prefetch_batches() == 1);

  REQUIRE(visit_order(loader) == before);
}

// ── stopped-epoch dereference (#280) ──────────────────────────────────────

// get_batch() returns an empty optional once the epoch has finished. Both
// Iterator::operator* and Iterator::move_batch() used to dereference that
// optional unconditionally — undefined behaviour rather than a diagnostic.
// They now throw, naming the batch index.
//
// No timing is involved in reaching that state: set_prefetch_batches() (like
// set_num_workers() and the destructor) calls stop(), which joins every worker
// and *then* clears the batch queue, so once it returns the queue is
// guaranteed empty and epoch_finished_ is guaranteed set. A dereference after
// that point deterministically finds nothing.

TEST_CASE("Dataloader iterator throws when dereferenced after the epoch stops",
          "[vision][dataloader]") {
  auto db = make_db(10);
  IndexDataset ds(db);

  Dataloader loader(ds, 4, false, 2);

  auto it = loader.begin(); // starts the epoch, but do not dereference yet
  loader.set_prefetch_batches(1); // stop(): joins workers, clears the queue

  REQUIRE_THROWS_AS(*it, std::runtime_error);
}

TEST_CASE("Dataloader move_batch throws when the epoch has stopped",
          "[vision][dataloader]") {
  auto db = make_db(10);
  IndexDataset ds(db);

  Dataloader loader(ds, 4, false, 2);

  auto it = loader.begin();
  loader.set_num_workers(1); // stop() again — same deterministic state

  REQUIRE_THROWS_AS(it.move_batch(), std::runtime_error);
}

TEST_CASE("Dataloader stopped-epoch error names the batch index",
          "[vision][dataloader]") {
  auto db = make_db(20);
  IndexDataset ds(db);

  Dataloader loader(ds, 4, false, 2);

  auto it = loader.begin();
  ++it;
  ++it; // batch 2 — a non-zero index, so the message cannot pass by accident
  loader.set_prefetch_batches(1);

  try {
    (void)*it;
    FAIL("dereferencing a stopped iterator did not throw");
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    REQUIRE(what.find("Dataloader") != std::string::npos);
    REQUIRE(what.find("batch 2") != std::string::npos);
  }
}

TEST_CASE("Dataloader entropy seeding still visits every sample once",
          "[vision][dataloader]") {
  auto db = make_db(30);
  IndexDataset ds(db);

  Dataloader loader(ds, 7, true, 2, 2, std::nullopt);

  auto order = visit_order(loader);
  std::sort(order.begin(), order.end());
  REQUIRE(order == iota_vector(30));
}

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

/* * Dataloader is a class that provides an iterable interface to a dataset.
 * It loads batches of data from the dataset in a separate thread and provides
 * them to the user when requested. It supports shuffling, multiple worker
 * threads, and prefetching batches. The user can iterate over the dataloader
 * using a range-based for loop or by manually creating an iterator. The
 * dataloader will automatically stop the worker threads when the iteration is
 * complete or when the dataloader is destroyed.
 * */
class Dataloader {
    public:
  using Pair = IDataset::Pair;
  using Batch = std::vector<Pair>;
  using BatchView = std::span<Pair>;

  /* * Constructs a Dataloader for the given dataset with the specified batch
   * size, shuffle option, number of worker threads, and number of prefetch
   * batches.
   * @param dataset The dataset to load data from.
   * @param batch_size The number of samples in each batch.
   * @param shuffle Whether to shuffle the dataset at the beginning of each
   * epoch.
   * @param num_workers The number of worker threads to use for loading batches.
   * @param prefetch_batches The number of batches to prefetch in the
   * background.
   * */
  Dataloader(IDataset &dataset, size_t batch_size, bool shuffle = false,
             size_t num_workers = 4, size_t prefetch_batches = 2);

  ~Dataloader();

  /* * Iterator is a class that provides an input iterator interface to the
   * Dataloader. It allows the user to iterate over the batches of data in the
   * dataloader using a range-based for loop or by manually creating an
   * iterator. The iterator will automatically load batches from the dataloader
   * as needed and will stop when the iteration is complete.
   * */
  class Iterator {
      public:
    using iterator_category = std::input_iterator_tag;
    using value_type = Batch;
    using difference_type = std::ptrdiff_t;
    using pointer = Batch *;
    using reference = Batch &;

    /* * Constructs an iterator for the given dataloader and batch index.
     * @param loader The dataloader to iterate over.
     * @param batch_idx The index of the batch to start iterating from.
     * */
    Iterator(Dataloader *loader, size_t batch_idx);

    /* * Copy constructor and copy assignment operator for the iterator. The
     * current batch is not copied and will be reloaded when the iterator is
     * dereferenced.
     * @param other The iterator to copy from.
     * @return A reference to the copied iterator.
     * */
    Iterator(const Iterator &other)
        : loader_(other.loader_), batch_idx_(other.batch_idx_),
          current_batch_(std::nullopt) {}

    /* * Copy assignment operator for the iterator. The current batch is not
     * copied and will be reloaded when the iterator is dereferenced.
     * @param other The iterator to copy from.
     * @return A reference to the copied iterator.
     * */
    Iterator &operator=(const Iterator &other) {
      if (this != &other) {
        loader_ = other.loader_;
        batch_idx_ = other.batch_idx_;
        current_batch_ = std::nullopt;
      }
      return *this;
    }

    /* * Move constructor and move assignment operator for the iterator. The
     * current batch is not moved and will be reloaded when the iterator is
     * dereferenced.
     * @param other The iterator to move from.
     * @return A reference to the moved iterator.
     * */
    Iterator(Iterator &&) = default;

    /* * Move assignment operator for the iterator. The current batch is not
     * moved and will be reloaded when the iterator is dereferenced.
     * @param other The iterator to move from.
     * @return A reference to the moved iterator.
     * */
    Iterator &operator=(Iterator &&) = default;

    /* * Dereference operator for the iterator. It returns a view of the current
     * batch of data. If the current batch is not loaded, it will be loaded
     * from the dataloader.
     * @return A view of the current batch of data.
     * */
    BatchView operator*() const;

    /* * Pre-increment operator for the iterator. It advances the iterator to
     * the next batch of data. If the next batch is not loaded, it will be
     * loaded from the dataloader.
     * @return A reference to the advanced iterator.
     * */
    Batch &&move_batch();

    /* * Pre-increment operator for the iterator. It advances the iterator to
     * the next batch of data. If the next batch is not loaded, it will be
     * loaded from the dataloader.
     * @return A reference to the advanced iterator.
     * */
    Iterator &operator++();

    /* * Post-increment operator for the iterator. It is deleted to prevent
     * inefficient copying of batches. Use the pre-increment operator instead.
     * */
    Iterator operator++(int) = delete;

    /* * Equality operator for the iterator. It checks if two iterators are
     * equal by comparing their dataloader pointers and batch indices.
     * @param other The iterator to compare with.
     * @return True if the iterators are equal, false otherwise.
     * */
    bool operator==(const Iterator &other) const;

    /* * Inequality operator for the iterator. It checks if two iterators are
     * not equal by comparing their dataloader pointers and batch indices.
     * @param other The iterator to compare with.
     * @return True if the iterators are not equal, false otherwise.
     * */
    bool operator!=(const Iterator &other) const;

      private:
    /* * The dataloader that this iterator belongs to. It is used to load
     * batches of data when the iterator is dereferenced or advanced.
     * */
    Dataloader *loader_;
    /* * The index of the current batch that this iterator points to. It is used
     * to determine which batch to load from the dataloader when the iterator is
     * dereferenced or advanced.
     * */
    size_t batch_idx_;
    /* * The current batch of data that this iterator points to. It is stored as
     * an optional value because it may not be loaded yet. When the iterator is
     * dereferenced or advanced, the current batch will be loaded from the
     * dataloader if it is not already loaded.
     * */
    mutable std::optional<Batch> current_batch_;
  };

  /* * Returns an iterator to the beginning of the dataloader. The iterator will
   * point to the first batch of data in the dataloader.
   * @return An iterator to the beginning of the dataloader.
   * */
  Iterator begin();

  /* * Returns an iterator to the end of the dataloader. The iterator will point
   * to one past the last batch of data in the dataloader.
   * @return An iterator to the end of the dataloader.
   * */
  Iterator end();

  /* * Returns the total number of batches in the dataloader. This is calculated
   * based on the size of the dataset and the batch size.
   * @return The total number of batches in the dataloader.
   * */
  size_t size() const;

  /* * Sets the number of worker threads to use for loading batches. This will
   * affect the performance of the dataloader, as more worker threads can load
   * batches in parallel, but may also increase the overhead of thread
   * management. The default number of worker threads is 4.
   * @param num_workers The number of worker threads to use for loading batches.
   * */
  void set_num_workers(size_t num_workers);

  /* * Sets the number of batches to prefetch in the background. This will
   * affect the performance of the dataloader, as more prefetch batches can
   * reduce the waiting time for batches to be loaded, but may also increase the
   * memory usage of the dataloader. The default number of prefetch batches is
   * 2.
   * @param prefetch_batches The number of batches to prefetch in the
   * background.
   * */
  void set_prefetch_batches(size_t prefetch_batches);

  /* * Returns the number of worker threads currently used for loading batches.
   * @return The number of worker threads currently used for loading batches.
   * */
  size_t get_num_workers() const;

  /* * Returns the number of batches currently prefetched in the background.
   * @return The number of batches currently prefetched in the background.
   * */
  size_t get_prefetch_batches() const;

    private:
  /* * Starts the worker threads for loading batches. This function is called at
   * the beginning of each epoch to initialize the worker threads and start
   * loading batches from the dataset. The worker threads will continue to load
   * batches in the background until the epoch is finished or the dataloader is
   * destroyed.
   * */
  void start_epoch();

  /* * Stops the worker threads for loading batches. This function is called at
   * the end of each epoch or when the dataloader is destroyed to signal the
   * worker threads to stop loading batches and exit. The worker threads will
   * check for this signal and exit gracefully when it is set.
   * */
  void stop();

  /* * The function that each worker thread runs to load batches of data from
   * the dataset. Each worker thread will continuously load batches of data from
   * the dataset and add them to the batch queue until the epoch is finished or
   * the dataloader is destroyed. The worker threads will synchronize access to
   * the batch queue using mutexes and condition variables to ensure thread
   * safety.
   * */
  void worker_thread();

  /* * Loads a batch of data from the dataset for the given batch index. This
   * function is called by the worker threads to load batches of data from the
   * dataset. It will calculate the indices of the samples in the batch based on
   * the batch index and batch size, and then load the corresponding samples
   * from the dataset. The loaded batch will be returned as a vector of pairs.
   * @param batch_idx The index of the batch to load.
   * @return A vector of pairs representing the loaded batch of data.
   * */
  Batch load_batch(size_t batch_idx);

  /* * Retrieves a batch of data from the batch queue for the given batch index.
   * This function is called by the iterator to retrieve batches of data from
   * the dataloader. It will check if the requested batch is already loaded in
   * the batch queue, and if so, it will return it. If the requested batch is
   * not loaded yet, it will wait for the worker threads to load it and add it
   * to the batch queue. The function will return an optional value, which will
   * be empty if the epoch is finished or if the dataloader is destroyed.
   * @param batch_idx The index of the batch to retrieve.
   * @return An optional value containing the retrieved batch of data, or empty
   * if the epoch is finished or if the dataloader is destroyed.
   * */
  std::optional<Batch> get_batch(size_t batch_idx);

  /* * The dataset that this dataloader loads data from. It is a reference to an
   * IDataset object, which provides the interface for accessing the samples in
   * the dataset. The dataloader will use this dataset to load batches of data
   * in the worker threads.
   * */
  IDataset &dataset_;

  /* * The batch size that this dataloader uses to load batches of data. It
   * determines how many samples are included in each batch that the dataloader
   * loads from the dataset. The batch size is set at the construction of the
   * dataloader and cannot be changed afterwards.
   * */
  size_t batch_size_;

  /* * Whether to shuffle the dataset at the beginning of each epoch. If true,
   * the dataloader will shuffle the indices of the samples in the dataset at
   * the beginning of each epoch, which will result in different batches being
   * loaded in each epoch. If false, the dataloader will load batches in a fixed
   * order based on the original order of the samples in the dataset.
   * */
  bool shuffle_;

  /* * The number of worker threads to use for loading batches. This determines
   * how many threads will be running in the background to load batches of data
   * from the dataset. More worker threads can load batches in parallel, but may
   * also increase the overhead of thread management. The default number of
   * worker threads is 4.
   * */
  size_t num_workers_;

  /* * The number of batches to prefetch in the background. This determines how
   * many batches will be loaded in advance by the worker threads while the user
   * is consuming the batches. More prefetch batches can reduce the waiting time
   * for batches to be loaded, but may also increase the memory usage of the
   * dataloader. The default number of prefetch batches is 2.
   * */
  size_t prefetch_batches_;

  /* * The total number of samples in the dataset. This is obtained from the
   * dataset object and is used to calculate the total number of batches in the
   * dataloader.
   * */
  size_t dataset_size_;

  /* * The total number of batches in the dataloader. This is calculated based
   * on the size of the dataset and the batch size. It is used to determine how
   * many batches are available for iteration in the dataloader.
   * */
  size_t num_batches_;

  /* * The indices of the samples in the dataset. This is a vector of size equal
   * to the number of samples in the dataset, containing the indices of the
   * samples in the original order. If shuffle is true, this vector will be
   * shuffled at the beginning of each epoch to provide different batches in
   * each epoch.
   * */
  std::vector<size_t> indices_;

  /* * The worker threads that are responsible for loading batches of data from
   * the dataset. This is a vector of threads that are created at the beginning
   * of each epoch and run the worker_thread function to load batches in the
   * background. The worker threads will continue to run until the epoch is
   * finished or the dataloader is destroyed.
   * */
  std::vector<std::thread> workers_;

  /* * The batch queue that holds the batches of data that have been loaded by
   * the worker threads. This is a queue of pairs, where each pair consists of a
   * batch index and the corresponding batch of data. The worker threads will
   * add batches to this queue as they load them, and the iterator will retrieve
   * batches from this queue when requested. The batch queue is synchronized
   * using mutexes and condition variables to ensure thread safety.
   * */
  std::queue<std::pair<size_t, Batch>> batch_queue_;

  /* * Mutex and condition variables for synchronizing access to the batch queue
   * and signaling the worker threads. The queue_mutex is used to protect access
   * to the batch_queue, while the queue_cv is used to signal the worker threads
   * when a new batch is added to the queue, and the ready_cv is used to signal
   * the iterator when a batch is ready to be retrieved from the queue.
   * */
  std::mutex queue_mutex_;

  /* * The condition variable used to signal the worker threads when a new batch
   * is added to the batch queue. The worker threads will wait on this condition
   * variable when they are idle and will be notified when a new batch is added
   * to the queue, allowing them to wake up and continue loading batches.
   * */
  std::condition_variable queue_cv_;

  /* * The condition variable used to signal the iterator when a batch is ready
   * to be retrieved from the batch queue. The iterator will wait on this
   * condition variable when it is waiting for a batch to be loaded, and will be
   * notified when a new batch is added to the queue, allowing it to wake up and
   * retrieve the batch.
   * */
  std::condition_variable ready_cv_;

  /* * Atomic flags for controlling the worker threads and the epoch state. The
   * stop_workers_ flag is used to signal the worker threads to stop loading
   * batches and exit when the epoch is finished or the dataloader is destroyed.
   * The epoch_finished_ flag is used to indicate whether the current epoch is
   * finished, which can be checked by the worker threads to determine when to
   * stop loading batches.
   * */
  std::atomic<bool> stop_workers_;

  /* * The epoch_finished_ flag is used to indicate whether the current epoch is
   * finished. It is set to true when the iteration over the dataloader is
   * complete or when the dataloader is destroyed, and it is checked by the
   * worker threads to determine when to stop loading batches. When this flag is
   * set to true, the worker threads will stop loading batches and exit
   * gracefully.
   * */
  std::atomic<bool> epoch_finished_;

  /* * The current batch index that the iterator is pointing to. This is used by
   * the iterator to determine which batch to load from the dataloader when it
   * is dereferenced or advanced. The current batch index is updated by the
   * iterator as it advances through the batches, and it is used to retrieve the
   * correct batch from the dataloader when requested.
   * */
  size_t current_batch_idx_;
};

} // namespace ReUseX::vision

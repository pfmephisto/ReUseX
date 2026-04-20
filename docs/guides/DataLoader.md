# DataLoader Documentation

## Overview

The `ReUseX::vision::Dataloader` class provides efficient multi-threaded data loading similar to PyTorch's DataLoader. It uses worker threads to preload batches in parallel, maximizing GPU/model utilization by minimizing data loading bottlenecks.

## Features

- **Multi-threaded loading**: Configurable number of worker threads for parallel data loading
- **Batch prefetching**: Asynchronous batch preparation to keep the training pipeline busy
- **STL-compatible iterators**: Standard C++ iterator interface for easy integration
- **Per-epoch shuffling**: Optional data shuffling for each training epoch
- **Flexible access patterns**: Both view-based and move-based batch access
- **Thread-safe**: Uses mutexes and condition variables for safe concurrent access

## Basic Usage

```cpp
#include <ReUseX/vision/Dataloader.hpp>
#include <ReUseX/vision/IDataset.hpp>

// Assume you have a dataset instance
MyDataset dataset("path/to/data.db");

// Create DataLoader
// Parameters: dataset, batch_size, shuffle, num_workers, prefetch_batches
ReUseX::vision::Dataloader loader(dataset, 
                                   /* batch_size */ 16,
                                   /* shuffle */ true,
                                   /* num_workers */ 4,
                                   /* prefetch_batches */ 2);

// Iterate through batches
for (auto batch_view : loader) {
    // batch_view is std::span<std::unique_ptr<IData>>
    // Process the batch...
    for (const auto& item : batch_view) {
        // Access data items
    }
}
```

## Advanced Usage

### Moving Batch Ownership

When you need to keep batch data beyond the current iteration:

```cpp
for (auto it = loader.begin(); it != loader.end(); ++it) {
    // Transfer ownership of the batch
    auto batch = it.move_batch();
    
    // batch is now std::vector<std::unique_ptr<IData>>
    // and remains valid after iterator advances
    
    // Store or process asynchronously...
}
```

### Configuring Worker Threads

```cpp
// Check current settings
size_t workers = loader.get_num_workers();
size_t prefetch = loader.get_prefetch_batches();

// Modify settings (stops and restarts workers)
loader.set_num_workers(8);
loader.set_prefetch_batches(3);
```

### View vs Move Semantics

**View access (`operator*`):**
- Returns `std::span<std::unique_ptr<IData>>`
- Lightweight, no copying
- Valid only until next `++` on iterator
- Best for immediate processing

**Move access (`move_batch()`):**
- Returns `std::vector<std::unique_ptr<IData>>`
- Transfers ownership
- Valid indefinitely
- Best for async processing or storage

## Architecture

### Threading Model

The DataLoader uses a producer-consumer pattern:

1. **Main thread**: Iterates through batches, consuming from queue
2. **Worker threads**: Load data and produce batches into queue
3. **Queue**: Thread-safe buffer of prefetched batches

```
┌─────────────┐
│ Main Thread │ ──(consume)──→ ┌───────────┐
└─────────────┘                 │   Queue   │
                                │  (Batches)│
┌─────────────┐                 └───────────┘
│  Worker 1   │ ──(produce)──→       ↑
├─────────────┤                       │
│  Worker 2   │ ──(produce)──────────┤
├─────────────┤                       │
│  Worker 3   │ ──(produce)──────────┤
└─────────────┘                       │
     ...                              │
```

### Synchronization

- **queue_mutex_**: Protects batch queue and control flags
- **queue_cv_**: Signals workers when queue has space
- **ready_cv_**: Signals main thread when batch is ready

### Shuffling

When `shuffle=true`, the DataLoader shuffles indices at the start of each epoch using `std::mt19937` random number generator.

## Configuration Recommendations

### Batch Size
- Depends on model capacity and memory
- Typical range: 8-64 for vision models
- Larger batches = better GPU utilization but more memory

### Number of Workers
- Default: 4 threads
- Rule of thumb: 2-4 workers per GPU
- More workers if I/O is bottleneck
- Fewer workers if CPU-bound preprocessing

### Prefetch Batches
- Default: 2 batches
- Higher values = more memory usage
- Lower values = potential pipeline stalls
- Sweet spot usually 2-3x number of workers

## Performance Tips

1. **Profile first**: Measure if data loading is actually the bottleneck
2. **Balance workers**: Too many workers can cause contention
3. **SSD storage**: Faster disk I/O significantly helps
4. **Cache datasets**: Load frequently-used data into memory
5. **Batch size**: Larger batches amortize loading overhead

## Example Integration

See `libs/reusex/src/vision/annotate.cpp` for a complete integration example with the vision pipeline.

## API Reference

### Constructor

```cpp
Dataloader(IDataset& dataset, 
          size_t batch_size,
          bool shuffle = false,
          size_t num_workers = 4,
          size_t prefetch_batches = 2)
```

### Iterator Methods

```cpp
Iterator begin()           // Start of epoch
Iterator end()             // End sentinel
size_t size() const        // Number of batches
```

### Configuration

```cpp
void set_num_workers(size_t num_workers)
void set_prefetch_batches(size_t prefetch_batches)
size_t get_num_workers() const
size_t get_prefetch_batches() const
```

### Iterator Operations

```cpp
BatchView operator*()      // Get view of current batch
Batch&& move_batch()       // Move ownership of batch
Iterator& operator++()     // Advance to next batch
bool operator==(const Iterator&) const
bool operator!=(const Iterator&) const
```

## Type Definitions

```cpp
using Batch = std::vector<std::unique_ptr<IData>>;
using BatchView = std::span<std::unique_ptr<IData>>;
```

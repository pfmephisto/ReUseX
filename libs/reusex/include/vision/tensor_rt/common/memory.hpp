#pragma once
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

namespace reusex::vision::tensor_rt::tensor {

class BaseMemory {
    public:
  BaseMemory() = default;
  BaseMemory(void *cpu, size_t cpu_bytes, void *gpu, size_t gpu_bytes);
  virtual ~BaseMemory();

  virtual void *gpu_realloc(size_t bytes);
  virtual void *cpu_realloc(size_t bytes);

  void release_gpu();
  void release_cpu();
  void release();

  // Status query
  inline size_t cpu_bytes() const { return cpu_bytes_; }
  inline size_t gpu_bytes() const { return gpu_bytes_; }

  // Get raw pointer (for computation)
  virtual inline void *get_gpu() const { return gpu_; }
  virtual inline void *get_cpu() const { return cpu_; }

  // Reference external memory (does not own)
  void reference(void *cpu, size_t cpu_bytes, void *gpu, size_t gpu_bytes);

  // [New]: Share memory of another Memory object (shared ownership)
  void set_shared_memory(const BaseMemory &other);

    protected:
  // Raw pointer kept for fast access, but lifetime managed by shared_ptr
  void *cpu_ = nullptr;
  size_t cpu_bytes_ = 0;
  size_t cpu_capacity_ = 0;

  void *gpu_ = nullptr;
  size_t gpu_bytes_ = 0;
  size_t gpu_capacity_ = 0;

  // Smart pointer manages lifetime
  std::shared_ptr<void> cpu_ptr_ = nullptr;
  std::shared_ptr<void> gpu_ptr_ = nullptr;
};

template <typename _DT> class Memory : public BaseMemory {
    public:
  Memory() = default;
  Memory(const Memory &other) { this->set_shared_memory(other); }
  Memory &operator=(const Memory &other) {
    if (this != &other)
      this->set_shared_memory(other);
    return *this;
  }

  virtual _DT *gpu(size_t size) {
    return (_DT *)BaseMemory::gpu_realloc(size * sizeof(_DT));
  }
  virtual _DT *cpu(size_t size) {
    return (_DT *)BaseMemory::cpu_realloc(size * sizeof(_DT));
  }

  inline size_t cpu_size() const { return cpu_bytes_ / sizeof(_DT); }
  inline size_t gpu_size() const { return gpu_bytes_ / sizeof(_DT); }

  virtual inline _DT *gpu() const { return (_DT *)gpu_; }
  virtual inline _DT *cpu() const { return (_DT *)cpu_; }
};

} // namespace reusex::vision::tensor_rt::tensor

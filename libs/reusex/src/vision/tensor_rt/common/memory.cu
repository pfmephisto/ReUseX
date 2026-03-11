#include <ReUseX/vision/tensor_rt/common/check.hpp>
#include <ReUseX/vision/tensor_rt/common/memory.hpp> // Assume the file name is memory.hpp
#include <cuda_runtime.h>

namespace ReUseX::vision::tensor_rt::tensor {

static size_t upbound(size_t n, size_t align) {
  return (n + align - 1) / align * align;
}

BaseMemory::BaseMemory(void *cpu, size_t cpu_bytes, void *gpu,
                       size_t gpu_bytes) {
  reference(cpu, cpu_bytes, gpu, gpu_bytes);
}

BaseMemory::~BaseMemory() { release(); }

// [New] Implement memory sharing logic
void BaseMemory::set_shared_memory(const BaseMemory &other) {
  // 1. Copy smart pointer (reference count +1)
  this->cpu_ptr_ = other.cpu_ptr_;
  this->gpu_ptr_ = other.gpu_ptr_;

  // 2. Synchronize raw pointer and metadata
  this->cpu_ = other.cpu_;
  this->cpu_bytes_ = other.cpu_bytes_;
  this->cpu_capacity_ = other.cpu_capacity_;

  this->gpu_ = other.gpu_;
  this->gpu_bytes_ = other.gpu_bytes_;
  this->gpu_capacity_ = other.gpu_capacity_;
}

void BaseMemory::reference(void *cpu, size_t cpu_bytes, void *gpu,
                           size_t gpu_bytes) {
  release();

  // Define a no-op deleter, because reference means referencing external
  // memory, not responsible for releasing
  auto no_op_deleter = [](void *) {};

  if (cpu && cpu_bytes > 0) {
    this->cpu_ptr_ = std::shared_ptr<void>(cpu, no_op_deleter);
    this->cpu_ = cpu;
    this->cpu_bytes_ = cpu_bytes;
    this->cpu_capacity_ = cpu_bytes;
  } else {
    this->cpu_ptr_.reset();
    this->cpu_ = nullptr;
    this->cpu_bytes_ = 0;
    this->cpu_capacity_ = 0;
  }

  if (gpu && gpu_bytes > 0) {
    this->gpu_ptr_ = std::shared_ptr<void>(gpu, no_op_deleter);
    this->gpu_ = gpu;
    this->gpu_bytes_ = gpu_bytes;
    this->gpu_capacity_ = gpu_bytes;
  } else {
    this->gpu_ptr_.reset();
    this->gpu_ = nullptr;
    this->gpu_bytes_ = 0;
    this->gpu_capacity_ = 0;
  }
}

void *BaseMemory::gpu_realloc(size_t bytes) {
  size_t size = upbound(bytes, 32);

  // If capacity is insufficient, reallocate
  if (gpu_capacity_ < size) {
    // Release the old one
    release_gpu();

    void *ptr = nullptr;
    checkRuntime(cudaMalloc(&ptr, size));

    // Create smart pointer, bind cudaFree as deleter
    gpu_ptr_ =
        std::shared_ptr<void>(ptr, [](void *p) { checkRuntime(cudaFree(p)); });

    gpu_ = ptr;
    gpu_capacity_ = size;
  }
  gpu_bytes_ = bytes;
  return gpu_;
}

void *BaseMemory::cpu_realloc(size_t bytes) {
  size_t size = upbound(bytes, 32);

  if (cpu_capacity_ < size) {
    release_cpu();

    void *ptr = nullptr;
    checkRuntime(cudaMallocHost(&ptr, size));
    Assert(ptr != nullptr);

    // Create smart pointer, bind cudaFreeHost as deleter
    cpu_ptr_ = std::shared_ptr<void>(
        ptr, [](void *p) { checkRuntime(cudaFreeHost(p)); });

    cpu_ = ptr;
    cpu_capacity_ = size;
  }
  cpu_bytes_ = bytes;
  return cpu_;
}

void BaseMemory::release_cpu() {
  // shared_ptr reset will decrease the reference count, and when it reaches
  // zero, the deleter is called automatically
  cpu_ptr_.reset();
  cpu_ = nullptr;
  cpu_capacity_ = 0;
  cpu_bytes_ = 0;
}

void BaseMemory::release_gpu() {
  gpu_ptr_.reset();
  gpu_ = nullptr;
  gpu_capacity_ = 0;
  gpu_bytes_ = 0;
}

void BaseMemory::release() {
  release_cpu();
  release_gpu();
}

} // namespace ReUseX::vision::tensor_rt::tensor

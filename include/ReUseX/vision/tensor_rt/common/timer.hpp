#pragma once
#include <ReUseX/vision/tensor_rt/common/check.hpp>
#include <spdlog/spdlog.h>

namespace ReUseX::vision::tensor_rt::nv {

class EventTimer {
    public:
  EventTimer() {
    checkRuntime(cudaEventCreate(&begin_));
    checkRuntime(cudaEventCreate(&end_));
  }

  virtual ~EventTimer() {
    checkRuntime(cudaEventDestroy(begin_));
    checkRuntime(cudaEventDestroy(end_));
  }

  void start(cudaStream_t stream = nullptr) {
    stream_ = stream;
    checkRuntime(cudaEventRecord(begin_, stream));
  }

  float stop(const char *prefix = "timer") {
    float times = 0;
    checkRuntime(cudaEventRecord(end_, stream_));
    checkRuntime(cudaEventSynchronize(end_));
    checkRuntime(cudaEventElapsedTime(&times, begin_, end_));
    spdlog::info("[⏰ {}] : {:.5f} ms", prefix, times);
    return times;
  }

    private:
  cudaStream_t stream_ = nullptr;
  cudaEvent_t begin_ = nullptr, end_ = nullptr;
};

}; // namespace ReUseX::vision::tensor_rt::nv

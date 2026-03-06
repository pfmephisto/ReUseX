#pragma once
#include <assert.h>
#include <cuda_runtime.h>
#include <spdlog/spdlog.h>
#include <stdarg.h>
#include <stdio.h>

#include <string>

namespace ReUseX::vision::tensor_rt::nv {

#define NVUNUSED2(a, b)                                                        \
  {                                                                            \
    (void)(a);                                                                 \
    (void)(b);                                                                 \
  }
#define NVUNUSED(a)                                                            \
  {                                                                            \
    (void)(a);                                                                 \
  }

#if DEBUG
#define checkRuntime(call)                                                     \
  ReUseX::vision::tensor_rt::nv::check_runtime(call, #call, __LINE__, __FILE__)
#define checkKernel(...)                                                       \
  [&] {                                                                        \
    __VA_ARGS__;                                                               \
    checkRuntime(cudaStreamSynchronize(nullptr));                              \
    return ReUseX::vision::tensor_rt::nv::check_runtime(                       \
        cudaGetLastError(), #__VA_ARGS__, __LINE__, __FILE__);                 \
  }()
#define dprintf printf
#else
#define checkRuntime(call)                                                     \
  ReUseX::vision::tensor_rt::nv::check_runtime(call, #call, __LINE__, __FILE__)
#define checkKernel(...)                                                       \
  do {                                                                         \
    __VA_ARGS__;                                                               \
    ReUseX::vision::tensor_rt::nv::check_runtime(                              \
        cudaPeekAtLastError(), #__VA_ARGS__, __LINE__, __FILE__);              \
  } while (false)
#define dprintf(...)
#endif

// #define Assertf(cond, fmt, ...) \
//   do { \
//     if (!(cond)) { \
//       fprintf(stderr, \
//               "Assert failed 💀. %s in file %s:%d, message: " fmt "\n",
// #cond,
//                   __FILE__, __LINE__, __VA_ARGS__); \
//       abort(); \
//     } \
//   } while (false)

// #define Asserts(cond, s) \
//   do { \
//     if (!(cond)) { \
//       fprintf(stderr, "Assert failed 💀. %s in file %s:%d, message: " s "\n",

//               #cond, __FILE__, __LINE__); \
//       abort(); \
//     } \
//   } while (false)

// #define Assert(cond) \
//   do { \
//     if (!(cond)) { \
//       fprintf(stderr, "Assert failed 💀. %s in file %s:%d\n", #cond,
//__FILE__, //               __LINE__); \
//       abort(); \
//     } \
//   } while (false)
//

static inline bool check_runtime(cudaError_t e, const char *call, int line,
                                 const char *file) {
  if (e != cudaSuccess) {
    fprintf(stderr,
            "CUDA Runtime error %s # %s, code = %s [ %d ] in file "
            "%s:%d\n",
            call, cudaGetErrorString(e), cudaGetErrorName(e), e, file, line);
    abort();
    return false;
  }
  return true;
}

}; // namespace ReUseX::vision::tensor_rt::nv

namespace ReUseX::vision::tensor_rt {

template <typename... Args>
constexpr void Assertf(bool cond, const char *fmt, Args... args) {
  do {
    if (!(cond)) {
      spdlog::error("Assert failed 💀. {} in file {}:{}, message: {}", cond,
                    __FILE__, __LINE__, /*__VA_ARGS__,*/ fmt);
      abort();
    }
  } while (false);
}

constexpr void Asserts(bool cond, const char *s) {
  if (!cond) {
    spdlog::error("Assert failed 💀. in file {}:{}, message: {}", __FILE__,
                  __LINE__, s);
    abort();
  }
}

constexpr void Assert(bool cond) {
  if (!cond) {
    spdlog::error("Assert failed 💀. in file {}:{}", __FILE__, __LINE__);
    abort();
  }
}
} // namespace ReUseX::vision::tensor_rt

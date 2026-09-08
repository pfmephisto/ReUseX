// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <cuda_runtime.h>

namespace reusex::vision::tensor_rt {
class AutoDevice {
    public:
  explicit AutoDevice(int device_id) {
    cudaGetDevice(&prev_device_);
    if (prev_device_ != device_id) {
      cudaSetDevice(device_id);
      switched_ = true;
    }
  }

  ~AutoDevice() {
    if (switched_) {
      cudaSetDevice(prev_device_);
    }
  }

  // Copy and assignment are prohibited
  AutoDevice(const AutoDevice &) = delete;
  AutoDevice &operator=(const AutoDevice &) = delete;

    private:
  int prev_device_ = 0;
  bool switched_ = false;
};
} // namespace reusex::vision::tensor_rt

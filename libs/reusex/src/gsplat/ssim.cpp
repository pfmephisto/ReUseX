// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ssim.hpp"

namespace reusex::gsplat::detail {

torch::Tensor gaussian_kernel1d(int window, double sigma,
                                const torch::TensorOptions &o) {
  auto x = torch::arange(window, o) - (window - 1) / 2.0;
  auto g = torch::exp(-x.pow(2) / (2.0 * sigma * sigma));
  return g / g.sum();
}

torch::Tensor ssim(const torch::Tensor &a, const torch::Tensor &b) {
  constexpr int window = 11;
  constexpr double sigma = 1.5;
  const int64_t channels = a.size(1);

  auto k1 = gaussian_kernel1d(window, sigma, a.options());
  auto k2 = k1.unsqueeze(1).mm(k1.unsqueeze(0)); // [w,w]
  auto kernel = k2.expand({channels, 1, window, window}).contiguous();

  auto conv = [&](const torch::Tensor &t) {
    return torch::conv2d(t, kernel, /*bias=*/{}, /*stride=*/1,
                         /*padding=*/window / 2, /*dilation=*/1,
                         /*groups=*/channels);
  };

  auto mu_a = conv(a);
  auto mu_b = conv(b);
  auto mu_a2 = mu_a * mu_a;
  auto mu_b2 = mu_b * mu_b;
  auto mu_ab = mu_a * mu_b;

  auto sigma_a2 = conv(a * a) - mu_a2;
  auto sigma_b2 = conv(b * b) - mu_b2;
  auto sigma_ab = conv(a * b) - mu_ab;

  constexpr double c1 = 0.01 * 0.01;
  constexpr double c2 = 0.03 * 0.03;
  auto num = (2 * mu_ab + c1) * (2 * sigma_ab + c2);
  auto den = (mu_a2 + mu_b2 + c1) * (sigma_a2 + sigma_b2 + c2);
  return (num / den).mean();
}

} // namespace reusex::gsplat::detail

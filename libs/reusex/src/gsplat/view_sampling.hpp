// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module-private: which views the trainer optimizes on, which it holds back,
// and how big the scene is.
//
// These three helpers decide what a training run is measured against, so they
// carry more consequence than their size suggests: `split_views` is the only
// thing that makes "held-out PSNR" a real claim, `stride_sample` decides
// whether an evaluation pass looks at the whole trajectory or at a contiguous
// prefix of it, and `scene_extent` scales the position learning rate — get it
// wrong by an order of magnitude and the run either freezes or explodes.
//
// They live here rather than in train.cpp's anonymous namespace because they
// are pure CPU (Eigen + std, no torch and no rasterizer) and therefore belong
// to `reusex_gsplat_common`, the half of the module that is built — and tested
// — even in the CUDA-less CI variant (#332). They stay module-private rather
// than public because they are an implementation detail of the training loop,
// not part of the trainer's API (docs/STANDARDS.md §2).
#pragma once

#include <reusex/gsplat/TrainingViews.hpp>

#include <cstddef>
#include <vector>

namespace reusex::gsplat::detail {

/// The two view index sets. Deterministic by construction: membership depends
/// only on a view's position in the list, never on the RNG, so two runs — or a
/// prune-only run and an MCMC run — evaluate on exactly the same images
/// (STANDARDS §6).
struct ViewSplit {
  std::vector<std::size_t> train;
  std::vector<std::size_t> holdout;
};

/// Hold back every @p holdout_every -th view (indices 0, N, 2N, …) from
/// training. @p holdout_every <= 0 puts every view in `train` and leaves
/// `holdout` empty.
///
/// Falls back to "train on everything, report no held-out metric" — with a
/// warning — when the requested split would leave nothing to train on.
ViewSplit split_views(std::size_t n_views, int holdout_every);

/// Sub-sample @p src to at most @p max_n entries by a uniform stride, keeping
/// the first entry. Deterministic, and it spreads the sample over the whole
/// trajectory rather than taking a contiguous prefix. Returns @p src unchanged
/// when it already fits the budget or when @p max_n <= 0.
std::vector<std::size_t> stride_sample(const std::vector<std::size_t> &src,
                                       int max_n);

/// Radius of the camera cloud — the reference scales the position learning
/// rate by it, because a step in metres only means something relative to how
/// big the scene is. Floored at 1e-3 m so a single-viewpoint capture does not
/// scale the learning rate to zero.
double scene_extent(const std::vector<TrainingView> &views);

} // namespace reusex::gsplat::detail

// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/IDataset.hpp"

#include <memory>

namespace reusex::vision {

/* IVideoModel is a stateful model interface for video / temporal-sequence
 * inference. It is intentionally distinct from IModel: whereas IModel::forward()
 * is a stateless, batch-oriented pass (safe to drive from a shuffled, multi
 * threaded Dataloader), IVideoModel carries an internal memory bank that couples
 * consecutive frames. The SAM 3.1 video tracker is the primary implementation.
 *
 * CONTRACT — callers MUST obey the following:
 *   - Feed frames in strict temporal order. The model conditions each frame on
 *     the memory accumulated from previous step() calls. Out-of-order or
 *     shuffled frames produce meaningless results.
 *   - NEVER drive an IVideoModel from the shuffled/multi-worker Dataloader. Use
 *     an ordered single-threaded loop over the dataset indices instead.
 *   - Call reset() at every sequence boundary (e.g. before the first frame of a
 *     new scan/sequence) to clear the memory bank.
 *
 * step() I/O contract:
 *   - input.first  : a TensorRTData (image + prompts + confidence_threshold).
 *   - input.second : the dataset index / sample id (echoed unchanged on output).
 *   - output.first : a TensorRTData whose .image is the CV_32S per-pixel label
 *                    image (background = -1, class ids 0+), exactly like
 *                    TensorRTSam3::forward().
 *   - output.second: echoes input.second so the caller can map results back to
 *                    the originating sample.
 */
class IVideoModel {
    public:
  /* Virtual destructor to ensure derived-class cleanup through a base pointer.
   */
  virtual ~IVideoModel() = default;

  /* Clears all temporal state (the memory bank) and returns the model to its
   * initial condition. Callers MUST invoke this at every sequence boundary
   * before feeding the first frame of a new sequence.
   */
  virtual void reset() = 0;

  /* Processes a single frame, conditioned on the memory accumulated from all
   * prior step() calls since the last reset(). See the class-level contract for
   * the exact input/output semantics.
   * @param in A Pair whose .first is a TensorRTData frame and whose .second is
   *           the sample index/id.
   * @return A Pair whose .first is a TensorRTData label image and whose .second
   *         echoes in.second.
   */
  virtual IDataset::Pair step(const IDataset::Pair &in) = 0;
};

} // namespace reusex::vision

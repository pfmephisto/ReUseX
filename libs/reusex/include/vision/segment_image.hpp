// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Single-frame SAM3 segmentation — the per-image building block factored out
// of segment_panorama so the GUI endpoint and CLI can call it directly (#409).
//
// Backend dispatch: tries TensorRT first (most common), then ONNX Runtime.
// Detection is runtime, via the IData subtype returned by forward(). At least
// one of REUSEX_USE_TENSORRT / REUSEX_USE_ONNX_RUNTIME must be compiled into
// the vision library for any result to be produced.

#pragma once

#include "reusex/vision/IModel.hpp"
#include "reusex/vision/sam3_prompt.hpp"

#include <opencv2/core.hpp>

#include <cstddef>
#include <vector>

namespace reusex::vision {

/// Segment a single BGR image with a SAM3 model.
///
/// @param model      A SAM3 model created via create_model_from_path().
///                   Must be TensorRT or ONNX; other types return an all-(-1)
///                   map.
/// @param image_bgr  Input BGR image (CV_8UC3, non-empty).
/// @param prompts    SAM3 prompts (text + optional boxes). Empty ⟹ model
/// default.
/// @param confidence Global detection confidence threshold [0,1].
///                   Per-prompt Sam3Prompt::confidence overrides when >= 0.
/// @return CV_32S label map the same size as @p image_bgr.
///         -1 = background; 0..N = class index matching prompt order.
///         Returns an empty Mat on empty input; all-(-1) on no detections.
///         When the backend cannot use boxes (see
///         clip_labels_to_prompt_boxes), boxed prompts are clipped to their
///         boxes and a warning is logged once per process.
/// Restrict each boxed prompt's pixels to its boxes — the fallback for a
/// backend that cannot feed boxes to SAM3 (ONNX, or TensorRT without
/// geometry-encoder.engine), where a boxed prompt would otherwise segment its
/// text concept across the whole image.
///
/// For prompt k (label value k in @p labels, the prompt-index numbering) with
/// at least one box: when it has "pos" boxes, its pixels outside the union of
/// those boxes become background (-1); pixels inside any of its "neg" boxes
/// become background too. Prompts without boxes and other labels are left
/// alone. Box coordinates are pixels of @p labels, clamped to the image.
/// Pixel clipping (not an IoU gate on whole detections) is the chosen rule:
/// the label map no longer carries per-detection boxes, and clipping keeps
/// the part of a detection the user actually boxed.
///
/// @param labels CV_32S label map, modified in place.
/// @return Number of pixels set to background.
std::size_t clip_labels_to_prompt_boxes(cv::Mat &labels,
                                        const std::vector<Sam3Prompt> &prompts);

cv::Mat segment_image(IModel &model, const cv::Mat &image_bgr,
                      const std::vector<Sam3Prompt> &prompts = {},
                      float confidence = 0.5f);

} // namespace reusex::vision

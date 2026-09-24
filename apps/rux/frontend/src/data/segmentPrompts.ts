// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pure coordinate math and prompt-building helpers for the SAM3 segment UI.
 *
 * No React, no side effects — only input → output.  The canvas event handlers
 * in SegmentPanel call these to convert from CSS display pixels (where the user
 * draws) to image pixel coordinates (what the backend expects).
 */

import type { FrameSegmentBox, FrameSegmentPrompt } from '../api/types';

/** A box in CSS display coordinates as drawn on the canvas. May be un-normalised. */
export interface DisplayBox {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
}

/**
 * Half-width of the synthetic box used to emulate a point click.
 *
 * SAM3 has no native point-prompt support (#409); a click is turned into a
 * 2×radius square centred on the click position.  The value mirrors the size
 * the OpenAPI spec's example uses (`x±8, y±8`).
 */
export const POINT_CLICK_RADIUS = 8;

/**
 * Ensure x1 ≤ x2 and y1 ≤ y2, as the backend requires.
 *
 * The user may drag in any direction; this normalises the result without
 * requiring the drawing code to track direction.
 */
export function normalizeDisplayBox(box: DisplayBox): DisplayBox {
  return {
    x1: Math.min(box.x1, box.x2),
    y1: Math.min(box.y1, box.y2),
    x2: Math.max(box.x1, box.x2),
    y2: Math.max(box.y1, box.y2),
  };
}

/**
 * Emulate a point click as a small square in display coordinates.
 *
 * The result is already normalised (x1 < x2, y1 < y2).
 */
export function clickToDisplayBox(cx: number, cy: number, radius = POINT_CLICK_RADIUS): DisplayBox {
  return { x1: cx - radius, y1: cy - radius, x2: cx + radius, y2: cy + radius };
}

/**
 * Convert a normalised display-pixel box to image-pixel coordinates.
 *
 * The image is displayed scaled inside a container; this maps from the CSS
 * pixel dimensions the canvas lives in back to the image's native resolution,
 * which is what the segmentation backend operates on.
 *
 * All four output coordinates are integers, clamped to [0, imageWidth/Height − 1].
 */
export function canvasToImageBox(
  box: DisplayBox,
  displayWidth: number,
  displayHeight: number,
  imageWidth: number,
  imageHeight: number,
): [number, number, number, number] {
  const sx = imageWidth / displayWidth;
  const sy = imageHeight / displayHeight;
  return [
    clamp(Math.round(box.x1 * sx), 0, imageWidth - 1),
    clamp(Math.round(box.y1 * sy), 0, imageHeight - 1),
    clamp(Math.round(box.x2 * sx), 0, imageWidth - 1),
    clamp(Math.round(box.y2 * sy), 0, imageHeight - 1),
  ];
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

/** Wrap an image-pixel box as a SAM3 box entry with a polarity tag. */
export function buildBoxEntry(
  imgBox: [number, number, number, number],
  polarity: 'pos' | 'neg' = 'pos',
): FrameSegmentBox {
  return [polarity, imgBox];
}

/**
 * Build one SAM3 prompt from a class name and zero or more image-pixel boxes.
 *
 * When `imgBoxes` is empty the `boxes` field is omitted entirely (the backend
 * treats absent and empty differently for text-only prompts).
 */
export function buildPrompt(
  text: string,
  imgBoxes: [number, number, number, number][],
): FrameSegmentPrompt {
  const prompt: FrameSegmentPrompt = { text };
  if (imgBoxes.length > 0) {
    prompt.boxes = imgBoxes.map((b) => buildBoxEntry(b));
  }
  return prompt;
}

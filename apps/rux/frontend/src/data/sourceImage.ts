// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pure helpers for the source-image cross-reference panel (#454).
 *
 * All functions here take plain data and return plain values — no React, no
 * fetch, no Three.js — so they can be unit-tested without DOM or GPU setup.
 */

import type { VisibleFrame } from '../api/types';

/**
 * Format a 0..1 visibility score as a human-readable percentage.
 *
 * The score is `1 - centrality`, so higher is better. `"90%"` is shown rather
 * than `"0.90"` because the user's question is "how central is this frame?",
 * not "what is the numerical centrality value?".
 */
export function formatScore(score: number): string {
  return `${Math.round(score * 100)}%`;
}

/**
 * The best (most central) frame from a ranked list, or null when the list is empty.
 *
 * The server already orders the list ascending by centrality, so the first
 * element is the best candidate source image. This function makes that
 * contract explicit so callers don't embed "index 0 = best" as a magic number.
 */
export function bestFrame(frames: VisibleFrame[]): VisibleFrame | null {
  return frames.length > 0 ? frames[0] : null;
}

/**
 * Human label for a frame's position in the ranked list.
 *
 * Rank 0 (the most central) is labelled "Best" to make the one-click shortcut
 * discoverable. Subsequent ranks use ordinal notation ("#2", "#3", …).
 */
export function rankLabel(index: number): string {
  return index === 0 ? 'Best' : `#${index + 1}`;
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { VisibleFrame } from '../api/types';
import {
  formatScore,
  bestFrame,
  rankLabel,
} from '../data/sourceImage';

// Minimal VisibleFrame helper
function frame(frame_id: number, score: number): VisibleFrame {
  return { frame_id, score, centrality: 1 - score, depth: 2.0, u: 320, v: 240 };
}

describe('formatScore', () => {
  it('renders a 0..1 score as a percentage with no decimal', () => {
    expect(formatScore(0.9)).toBe('90%');
    expect(formatScore(0.0)).toBe('0%');
    expect(formatScore(1.0)).toBe('100%');
  });

  it('rounds to the nearest percent', () => {
    expect(formatScore(0.999)).toBe('100%');
    expect(formatScore(0.505)).toBe('51%');
  });
});

describe('bestFrame', () => {
  it('returns null for an empty list', () => {
    expect(bestFrame([])).toBeNull();
  });

  it('returns the first frame (already ranked most-central first)', () => {
    const frames = [frame(42, 0.9), frame(7, 0.6)];
    expect(bestFrame(frames)?.frame_id).toBe(42);
  });
});

describe('rankLabel', () => {
  it('labels rank 0 as "Best"', () => {
    expect(rankLabel(0)).toBe('Best');
  });

  it('labels subsequent ranks as ordinal numbers', () => {
    expect(rankLabel(1)).toBe('#2');
    expect(rankLabel(9)).toBe('#10');
  });
});

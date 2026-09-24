// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';
import {
  POINT_CLICK_RADIUS,
  buildBoxEntry,
  buildPrompt,
  canvasToImageBox,
  clickToDisplayBox,
  normalizeDisplayBox,
} from '../data/segmentPrompts';

describe('normalizeDisplayBox', () => {
  it('is a no-op for an already-normalised box', () => {
    const box = { x1: 10, y1: 20, x2: 100, y2: 200 };
    expect(normalizeDisplayBox(box)).toEqual(box);
  });

  it('swaps x when x1 > x2', () => {
    expect(normalizeDisplayBox({ x1: 100, y1: 20, x2: 10, y2: 200 })).toEqual({
      x1: 10,
      y1: 20,
      x2: 100,
      y2: 200,
    });
  });

  it('swaps y when y1 > y2', () => {
    expect(normalizeDisplayBox({ x1: 10, y1: 200, x2: 100, y2: 20 })).toEqual({
      x1: 10,
      y1: 20,
      x2: 100,
      y2: 200,
    });
  });

  it('handles a zero-area point', () => {
    expect(normalizeDisplayBox({ x1: 50, y1: 50, x2: 50, y2: 50 })).toEqual({
      x1: 50,
      y1: 50,
      x2: 50,
      y2: 50,
    });
  });
});

describe('clickToDisplayBox', () => {
  it('creates a centred square with the default POINT_CLICK_RADIUS', () => {
    const box = clickToDisplayBox(50, 60);
    expect(box).toEqual({
      x1: 50 - POINT_CLICK_RADIUS,
      y1: 60 - POINT_CLICK_RADIUS,
      x2: 50 + POINT_CLICK_RADIUS,
      y2: 60 + POINT_CLICK_RADIUS,
    });
    expect(box.x2 - box.x1).toBe(POINT_CLICK_RADIUS * 2);
    expect(box.y2 - box.y1).toBe(POINT_CLICK_RADIUS * 2);
  });

  it('accepts a custom radius', () => {
    expect(clickToDisplayBox(0, 0, 4)).toEqual({ x1: -4, y1: -4, x2: 4, y2: 4 });
  });

  it('produces a normalised box (x1 ≤ x2, y1 ≤ y2)', () => {
    const box = clickToDisplayBox(30, 20, 8);
    expect(box.x1).toBeLessThanOrEqual(box.x2);
    expect(box.y1).toBeLessThanOrEqual(box.y2);
  });
});

describe('canvasToImageBox', () => {
  it('is an identity when display size equals image size', () => {
    const box = { x1: 10, y1: 20, x2: 100, y2: 200 };
    expect(canvasToImageBox(box, 640, 480, 640, 480)).toEqual([10, 20, 100, 200]);
  });

  it('scales up when the image is twice as large as the display', () => {
    const box = { x1: 50, y1: 50, x2: 100, y2: 100 };
    expect(canvasToImageBox(box, 320, 240, 640, 480)).toEqual([100, 100, 200, 200]);
  });

  it('scales down when the image is smaller than the display', () => {
    const box = { x1: 100, y1: 80, x2: 200, y2: 160 };
    expect(canvasToImageBox(box, 640, 480, 320, 240)).toEqual([50, 40, 100, 80]);
  });

  it('clamps out-of-bounds coordinates to the image extents', () => {
    const box = { x1: -10, y1: -5, x2: 9999, y2: 8888 };
    expect(canvasToImageBox(box, 640, 480, 640, 480)).toEqual([0, 0, 639, 479]);
  });

  it('returns integer pixel coordinates after rounding', () => {
    const box = { x1: 0.4, y1: 0.6, x2: 1.4, y2: 1.6 };
    const result = canvasToImageBox(box, 100, 100, 100, 100);
    expect(result.every(Number.isInteger)).toBe(true);
  });

  it('rounds fractional scaled coordinates', () => {
    // display 100px wide → image 300px; each display px = 3 image px
    const box = { x1: 10, y1: 10, x2: 11, y2: 11 };
    const [x1, y1, x2, y2] = canvasToImageBox(box, 100, 100, 300, 300);
    expect(x1).toBe(30);
    expect(y1).toBe(30);
    expect(x2).toBe(33);
    expect(y2).toBe(33);
  });
});

describe('buildBoxEntry', () => {
  it('wraps a box as a positive entry by default', () => {
    expect(buildBoxEntry([10, 20, 100, 200])).toEqual(['pos', [10, 20, 100, 200]]);
  });

  it('wraps a box as a negative entry when requested', () => {
    expect(buildBoxEntry([0, 0, 50, 50], 'neg')).toEqual(['neg', [0, 0, 50, 50]]);
  });
});

describe('buildPrompt', () => {
  it('creates a text-only prompt when no boxes are supplied', () => {
    const prompt = buildPrompt('wall', []);
    expect(prompt).toEqual({ text: 'wall' });
    expect(prompt.boxes).toBeUndefined();
  });

  it('includes boxes when supplied', () => {
    const prompt = buildPrompt('floor', [
      [0, 0, 100, 100],
      [200, 200, 300, 300],
    ]);
    expect(prompt.text).toBe('floor');
    expect(prompt.boxes).toHaveLength(2);
    expect(prompt.boxes![0]).toEqual(['pos', [0, 0, 100, 100]]);
    expect(prompt.boxes![1]).toEqual(['pos', [200, 200, 300, 300]]);
  });

  it('allows empty text with boxes (the backend accepts it)', () => {
    const prompt = buildPrompt('', [[10, 10, 50, 50]]);
    expect(prompt.text).toBe('');
    expect(prompt.boxes).toHaveLength(1);
  });

  it('all boxes default to positive polarity', () => {
    const prompt = buildPrompt('window', [[0, 0, 60, 60]]);
    expect(prompt.boxes![0][0]).toBe('pos');
  });
});

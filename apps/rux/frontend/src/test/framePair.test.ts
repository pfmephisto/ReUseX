// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  DESCRIPTOR_METHOD_LABELS,
  DESCRIPTOR_METHODS,
  IMAGE_KIND_LABELS,
  IMAGE_KINDS,
  parseFrameId,
} from '../data/framePair';

describe('parseFrameId', () => {
  it('parses a valid non-negative integer string', () => {
    expect(parseFrameId('0')).toBe(0);
    expect(parseFrameId('1')).toBe(1);
    expect(parseFrameId('1997')).toBe(1997);
    expect(parseFrameId('999999')).toBe(999999);
  });

  it('returns null for absent or empty values', () => {
    expect(parseFrameId(null)).toBeNull();
    expect(parseFrameId(undefined)).toBeNull();
    expect(parseFrameId('')).toBeNull();
  });

  it('returns null for non-integer strings', () => {
    expect(parseFrameId('abc')).toBeNull();
    expect(parseFrameId('1.5')).toBeNull();
    expect(parseFrameId('1e3')).toBeNull(); // 1000 but notation not integer URL param
    expect(parseFrameId('NaN')).toBeNull();
    expect(parseFrameId('Infinity')).toBeNull();
  });

  it('returns null for negative integers', () => {
    // Frame IDs are non-negative; -1 is not a valid frame id.
    expect(parseFrameId('-1')).toBeNull();
    expect(parseFrameId('-100')).toBeNull();
  });

  it('returns null for integers beyond MAX_SAFE_INTEGER', () => {
    // Number() loses precision for very large digit strings; reject them.
    expect(parseFrameId('99999999999999999999')).toBeNull();
    expect(parseFrameId('9007199254740992')).toBeNull(); // MAX_SAFE_INTEGER + 1
  });

  it('is independent for each side of the pair', () => {
    // a and b are parsed with the same function; independence means one null
    // does not affect the other.
    expect(parseFrameId('42')).toBe(42);
    expect(parseFrameId(null)).toBeNull();
  });
});

describe('DESCRIPTOR_METHODS', () => {
  it('contains the three documented methods', () => {
    expect(DESCRIPTOR_METHODS).toContain('orb');
    expect(DESCRIPTOR_METHODS).toContain('sift');
    expect(DESCRIPTOR_METHODS).toContain('akaze');
    expect(DESCRIPTOR_METHODS).toHaveLength(3);
  });

  it('has a label for every method', () => {
    for (const method of DESCRIPTOR_METHODS) {
      expect(typeof DESCRIPTOR_METHOD_LABELS[method]).toBe('string');
      expect(DESCRIPTOR_METHOD_LABELS[method].length).toBeGreaterThan(0);
    }
  });
});

describe('IMAGE_KINDS', () => {
  it('contains all four image kinds', () => {
    expect(IMAGE_KINDS).toContain('color');
    expect(IMAGE_KINDS).toContain('depth');
    expect(IMAGE_KINDS).toContain('confidence');
    expect(IMAGE_KINDS).toContain('segmentation');
    expect(IMAGE_KINDS).toHaveLength(4);
  });

  it('has a label for every kind', () => {
    for (const kind of IMAGE_KINDS) {
      expect(typeof IMAGE_KIND_LABELS[kind]).toBe('string');
      expect(IMAGE_KIND_LABELS[kind].length).toBeGreaterThan(0);
    }
  });
});

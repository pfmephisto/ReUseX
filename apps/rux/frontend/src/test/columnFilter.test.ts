// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { PropertyDefinition } from '../api/types';
import {
  countActiveFilters,
  isFilterActive,
  matchesColumnFilter,
  type ColumnFilters,
} from '../components/columnFilterHelpers';

function col(type: PropertyDefinition['type'], options?: string[]): PropertyDefinition {
  return { id: 'c1', name: 'Col', type, sort_order: 0, options };
}

// ------------------------------------------------------------------ isFilterActive

describe('isFilterActive', () => {
  it('null is never active (boolean any-filter)', () => {
    expect(isFilterActive(null)).toBe(false);
  });
  it('empty string is not active', () => {
    expect(isFilterActive('')).toBe(false);
  });
  it('non-empty string is active', () => {
    expect(isFilterActive('hello')).toBe(true);
  });
  it('empty array is not active', () => {
    expect(isFilterActive([])).toBe(false);
  });
  it('non-empty array is active', () => {
    expect(isFilterActive(['a'])).toBe(true);
  });
  it('object with no bounds is not active', () => {
    expect(isFilterActive({})).toBe(false);
  });
  it('object with min is active', () => {
    expect(isFilterActive({ min: 0 })).toBe(true);
  });
  it('object with max is active', () => {
    expect(isFilterActive({ max: 100 })).toBe(true);
  });
  it('object with from is active', () => {
    expect(isFilterActive({ from: '2026-01-01' })).toBe(true);
  });
});

// ------------------------------------------------------------------ text filter

describe('matchesColumnFilter – text', () => {
  const c = col('text');
  it('inactive filter always passes', () => {
    expect(matchesColumnFilter(c, 'hello', '')).toBe(true);
    expect(matchesColumnFilter(c, 'hello', undefined)).toBe(true);
  });
  it('substring match is case-insensitive', () => {
    expect(matchesColumnFilter(c, 'Hello World', 'world')).toBe(true);
    expect(matchesColumnFilter(c, 'Hello World', 'HELLO')).toBe(true);
  });
  it('no match returns false', () => {
    expect(matchesColumnFilter(c, 'concrete', 'brick')).toBe(false);
  });
  it('undefined value treated as empty string', () => {
    expect(matchesColumnFilter(c, undefined, 'any')).toBe(false);
  });
});

// ------------------------------------------------------------------ url filter

describe('matchesColumnFilter – url', () => {
  const c = col('url');
  it('matches substring of url', () => {
    expect(matchesColumnFilter(c, 'https://example.com/page', 'example')).toBe(true);
  });
  it('no match', () => {
    expect(matchesColumnFilter(c, 'https://example.com', 'github')).toBe(false);
  });
  it('inactive filter passes', () => {
    expect(matchesColumnFilter(c, 'https://anything.com', '')).toBe(true);
  });
});

// ------------------------------------------------------------------ number filter

describe('matchesColumnFilter – number', () => {
  const c = col('number');
  it('passes when within range', () => {
    expect(matchesColumnFilter(c, '50', { min: 10, max: 100 })).toBe(true);
  });
  it('fails when below min', () => {
    expect(matchesColumnFilter(c, '5', { min: 10 })).toBe(false);
  });
  it('fails when above max', () => {
    expect(matchesColumnFilter(c, '200', { max: 100 })).toBe(false);
  });
  it('passes at boundary (min)', () => {
    expect(matchesColumnFilter(c, '10', { min: 10 })).toBe(true);
  });
  it('passes at boundary (max)', () => {
    expect(matchesColumnFilter(c, '100', { max: 100 })).toBe(true);
  });
  it('empty value fails when min or max is set', () => {
    expect(matchesColumnFilter(c, '', { min: 0 })).toBe(false);
    expect(matchesColumnFilter(c, undefined, { max: 10 })).toBe(false);
  });
  it('empty value passes when no bounds set', () => {
    expect(matchesColumnFilter(c, '', {})).toBe(true);
  });
});

// ------------------------------------------------------------------ date filter

describe('matchesColumnFilter – date', () => {
  const c = col('date');
  it('passes when within range', () => {
    expect(matchesColumnFilter(c, '2026-06-15', { from: '2026-01-01', to: '2026-12-31' })).toBe(true);
  });
  it('fails when before from', () => {
    expect(matchesColumnFilter(c, '2025-12-31', { from: '2026-01-01' })).toBe(false);
  });
  it('fails when after to', () => {
    expect(matchesColumnFilter(c, '2027-01-01', { to: '2026-12-31' })).toBe(false);
  });
  it('passes at from boundary', () => {
    expect(matchesColumnFilter(c, '2026-01-01', { from: '2026-01-01' })).toBe(true);
  });
});

// ------------------------------------------------------------------ boolean filter

describe('matchesColumnFilter – boolean', () => {
  const c = col('boolean');
  it('null filter passes everything', () => {
    expect(matchesColumnFilter(c, 'true', null)).toBe(true);
    expect(matchesColumnFilter(c, 'false', null)).toBe(true);
    expect(matchesColumnFilter(c, '', null)).toBe(true);
  });
  it('"true" filter matches only true', () => {
    expect(matchesColumnFilter(c, 'true', 'true')).toBe(true);
    expect(matchesColumnFilter(c, 'false', 'true')).toBe(false);
  });
  it('"false" filter matches only false', () => {
    expect(matchesColumnFilter(c, 'false', 'false')).toBe(true);
    expect(matchesColumnFilter(c, 'true', 'false')).toBe(false);
  });
});

// ------------------------------------------------------------------ select filter

describe('matchesColumnFilter – select', () => {
  const c = col('select', ['Red', 'Green', 'Blue']);
  it('empty membership passes all', () => {
    expect(matchesColumnFilter(c, 'Red', [])).toBe(true);
  });
  it('passes when value is in membership set', () => {
    expect(matchesColumnFilter(c, 'Green', ['Green', 'Blue'])).toBe(true);
  });
  it('fails when value is not in membership set', () => {
    expect(matchesColumnFilter(c, 'Red', ['Green', 'Blue'])).toBe(false);
  });
});

// ------------------------------------------------------------------ multiselect filter

describe('matchesColumnFilter – multiselect', () => {
  const c = col('multiselect', ['A', 'B', 'C']);
  it('empty membership passes all', () => {
    expect(matchesColumnFilter(c, JSON.stringify(['A', 'B']), [])).toBe(true);
  });
  it('passes when any selected value overlaps with filter', () => {
    expect(matchesColumnFilter(c, JSON.stringify(['A', 'C']), ['B', 'C'])).toBe(true);
  });
  it('fails when no overlap', () => {
    expect(matchesColumnFilter(c, JSON.stringify(['A']), ['B', 'C'])).toBe(false);
  });
  it('handles non-JSON value gracefully', () => {
    expect(matchesColumnFilter(c, 'A', ['A'])).toBe(true);
  });
});

// ------------------------------------------------------------------ countActiveFilters

describe('countActiveFilters', () => {
  it('counts only active filters', () => {
    const filters: ColumnFilters = {
      col1: 'hello',
      col2: '',
      col3: ['red'],
      col4: [],
      col5: { min: 5 },
      col6: {},
    };
    expect(countActiveFilters(filters)).toBe(3); // col1, col3, col5
  });

  it('returns 0 when all filters are empty', () => {
    const filters: ColumnFilters = { col1: '', col2: [], col3: null };
    expect(countActiveFilters(filters)).toBe(0);
  });
});

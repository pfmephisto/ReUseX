// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  configToSelection,
  selectionToConfig,
  selectionToQueryParam,
  STRUCTURAL_COLUMNS,
} from '../data/csvExport';

const ALL_KEYS = STRUCTURAL_COLUMNS.map((c) => c.key);

describe('selectionToConfig', () => {
  it('preserves allKeys order regardless of Set insertion order', () => {
    const sel = new Set(['id', 'kind']);
    expect(selectionToConfig(sel, ALL_KEYS).columns).toEqual(['kind', 'id']);
  });

  it('includes only selected keys', () => {
    const sel = new Set(['component_name', 'component_type']);
    expect(selectionToConfig(sel, ALL_KEYS).columns).toEqual([
      'component_name',
      'component_type',
    ]);
  });

  it('returns empty array when nothing is selected', () => {
    expect(selectionToConfig(new Set(), ALL_KEYS).columns).toEqual([]);
  });

  it('returns all keys when all selected', () => {
    expect(selectionToConfig(new Set(ALL_KEYS), ALL_KEYS).columns).toEqual(ALL_KEYS);
  });

  it('ignores keys not in allKeys', () => {
    const sel = new Set(['kind', 'unknown_col']);
    expect(selectionToConfig(sel, ALL_KEYS).columns).toEqual(['kind']);
  });
});

describe('configToSelection', () => {
  it('selects all keys when config is undefined', () => {
    expect(configToSelection(undefined, ALL_KEYS)).toEqual(new Set(ALL_KEYS));
  });

  it('selects all keys when config.columns is empty', () => {
    expect(configToSelection({ columns: [] }, ALL_KEYS)).toEqual(new Set(ALL_KEYS));
  });

  it('selects all keys when config is null', () => {
    expect(configToSelection(null, ALL_KEYS)).toEqual(new Set(ALL_KEYS));
  });

  it('returns the specified columns as a Set', () => {
    const result = configToSelection({ columns: ['kind', 'id'] }, ALL_KEYS);
    expect(result).toEqual(new Set(['kind', 'id']));
  });

  it('silently discards unknown columns', () => {
    const result = configToSelection({ columns: ['kind', 'not_a_real_column'] }, ALL_KEYS);
    expect(result).toEqual(new Set(['kind']));
  });

  it('round-trips with selectionToConfig', () => {
    const original = new Set(['kind', 'id', 'component_name', 'linked_instance']);
    const config = selectionToConfig(original, ALL_KEYS);
    const recovered = configToSelection(config, ALL_KEYS);
    expect(recovered).toEqual(original);
  });
});

describe('selectionToQueryParam', () => {
  it('returns undefined when all keys are selected', () => {
    expect(selectionToQueryParam(new Set(ALL_KEYS), ALL_KEYS)).toBeUndefined();
  });

  it('returns undefined when nothing is selected', () => {
    expect(selectionToQueryParam(new Set(), ALL_KEYS)).toBeUndefined();
  });

  it('returns comma-joined keys in allKeys order', () => {
    const sel = new Set(['id', 'kind']);
    expect(selectionToQueryParam(sel, ALL_KEYS)).toBe('kind,id');
  });

  it('handles a single column', () => {
    expect(selectionToQueryParam(new Set(['notes']), ALL_KEYS)).toBe('notes');
  });
});

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  applyLabelPatch,
  applyPropertyPatch,
  diffLabels,
  diffProperties,
  duplicateRowKeys,
  isEmptyPatch,
  isLegendEditable,
  labelIssues,
  propertyIssues,
  propertyRows,
  rowsToDraft,
} from '../data/patch';

const legend = { '1': 'wall', '2': 'floor', '3': 'ceiling' };

describe('diffLabels', () => {
  it('sends only the ids whose name actually changed', () => {
    expect(diffLabels(legend, { ...legend, '2': 'slab' })).toEqual({ '2': 'slab' });
  });

  it('omits a name edited and then typed back to its original', () => {
    // Otherwise undoing a typo still takes the project's writer lock.
    expect(diffLabels(legend, { ...legend })).toEqual({});
  });

  it('omits an id that is not already in the legend', () => {
    // The server 400s on it; sending it would fail the whole patch, taking the
    // user's other, valid renames down with it.
    expect(diffLabels(legend, { ...legend, '9': 'ghost' })).toEqual({});
  });

  it('carries several renames in one patch', () => {
    const patch = diffLabels(legend, { '1': 'partition', '2': 'floor', '3': 'soffit' });
    expect(patch).toEqual({ '1': 'partition', '3': 'soffit' });
  });
});

describe('labelIssues', () => {
  it('accepts an untouched legend', () => {
    expect(labelIssues(legend, legend)).toEqual([]);
  });

  it('rejects an emptied or whitespace-only name', () => {
    expect(labelIssues(legend, { ...legend, '1': '' })).toHaveLength(1);
    expect(labelIssues(legend, { ...legend, '1': '   ' })).toHaveLength(1);
  });

  it('explains that a class can be renamed but not created', () => {
    const issues = labelIssues(legend, { ...legend, '42': 'new class' });
    expect(issues).toHaveLength(1);
    expect(issues[0].id).toBe('42');
    expect(issues[0].message).toMatch(/renamed, not created/);
  });
});

describe('isLegendEditable', () => {
  it('refuses the instances cloud, whose names encode the semantic class', () => {
    expect(isLegendEditable('instances')).toBe(false);
  });

  it('allows every other label cloud', () => {
    for (const cloud of ['labels', 'planes', 'rooms']) {
      expect(isLegendEditable(cloud)).toBe(true);
    }
  });
});

const stored = { Condition: 'Good', Quantity: '3 units' };

describe('diffProperties', () => {
  it('sends a changed value as a string', () => {
    expect(diffProperties(stored, { ...stored, Condition: 'Fair' })).toEqual({
      Condition: 'Fair',
    });
  });

  it('sends a removed key as null rather than dropping it', () => {
    // An omitted key means "leave it alone" to the server, so a deletion that
    // was merely omitted would silently not happen.
    expect(diffProperties(stored, { Condition: 'Good' })).toEqual({ Quantity: null });
  });

  it('sends a brand-new property as a string', () => {
    expect(diffProperties(stored, { ...stored, 'Fire rating': 'EI30' })).toEqual({
      'Fire rating': 'EI30',
    });
  });

  it('omits a value edited and then typed back to its original', () => {
    expect(diffProperties(stored, { ...stored })).toEqual({});
  });

  it('distinguishes clearing a property from storing an empty one', () => {
    // Absent from the draft = delete; present but empty = store "".
    expect(diffProperties(stored, { Quantity: '3 units' })).toEqual({ Condition: null });
    expect(diffProperties(stored, { ...stored, Condition: '' })).toEqual({ Condition: '' });
  });

  it('combines a set, a clear and an addition in one patch', () => {
    const patch = diffProperties(stored, { Condition: 'Refurbished', Reuse: 'High' });
    expect(patch).toEqual({ Condition: 'Refurbished', Reuse: 'High', Quantity: null });
  });
});

describe('propertyIssues', () => {
  it('accepts an empty value, which is a legitimate stored value', () => {
    expect(propertyIssues({ Condition: '' })).toEqual([]);
  });

  it('rejects a blank property name, which the user could never find again', () => {
    expect(propertyIssues({ '   ': 'x' })).toHaveLength(1);
  });
});

describe('property rows', () => {
  it('seeds stable ids from the stored names so the form does not remount', () => {
    const rows = propertyRows(stored);
    expect(rows.map((r) => r.id)).toEqual(['stored:Condition', 'stored:Quantity']);
    expect(rows.every((r) => !r.added)).toBe(true);
    // Re-seeding after a save must not change identity.
    expect(propertyRows(stored).map((r) => r.id)).toEqual(rows.map((r) => r.id));
  });

  it('drops a half-typed row rather than sending an empty property name', () => {
    const draft = rowsToDraft([
      { id: 'a', key: 'Condition', value: 'Good', added: false },
      { id: 'b', key: '  ', value: 'orphan', added: true },
    ]);
    expect(draft).toEqual({ Condition: 'Good' });
  });

  it('names every duplicated key so the form can refuse to save', () => {
    const rows = [
      { id: 'a', key: 'Condition', value: '1', added: false },
      { id: 'b', key: 'Condition', value: '2', added: true },
      { id: 'c', key: 'Quantity', value: '3', added: false },
    ];
    expect(duplicateRowKeys(rows)).toEqual(['Condition']);
  });

  it('does not count blank rows as duplicates of each other', () => {
    const rows = [
      { id: 'a', key: '', value: '', added: true },
      { id: 'b', key: '', value: '', added: true },
    ];
    expect(duplicateRowKeys(rows)).toEqual([]);
  });
});

describe('optimistic application', () => {
  it('applies a property patch the way the server will', () => {
    const next = applyPropertyPatch(stored, { Condition: 'Fair', Quantity: null, New: 'x' });
    expect(next).toEqual({ Condition: 'Fair', New: 'x' });
  });

  it('leaves the original untouched so a rollback has something to restore', () => {
    const before = { ...stored };
    applyPropertyPatch(stored, { Condition: null });
    expect(stored).toEqual(before);
  });

  it('applies a label patch as a rename, never a removal', () => {
    const next = applyLabelPatch(legend, { '2': 'slab' });
    expect(next).toEqual({ '1': 'wall', '2': 'slab', '3': 'ceiling' });
  });
});

describe('isEmptyPatch', () => {
  it('is true for a patch that would change nothing', () => {
    expect(isEmptyPatch({})).toBe(true);
  });

  it('is false for a patch that only clears something', () => {
    // `null` is an edit, not an absence.
    expect(isEmptyPatch({ Quantity: null })).toBe(false);
  });
});

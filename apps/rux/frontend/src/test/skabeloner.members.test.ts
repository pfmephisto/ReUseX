// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { ResourceKey, TemplateMember } from '../api/types';
import {
  addKey,
  categoryCounts,
  hasCategory,
  memberId,
  memberLabel,
  moveMember,
  removeMemberAt,
  resolveMembers,
  searchKeys,
  toggleCategory,
} from '../skabeloner/members';

function key(id: string, label: string, category: string): ResourceKey {
  return {
    id,
    label,
    category,
    scope: id.startsWith('sys:') ? 'part' : 'type',
    data_type: 'text',
    unit: null,
    options: [],
    editable: true,
  };
}

const KEYS: ResourceKey[] = [
  key('sys:name', 'Betegnelse', 'Kortlægning'),
  key('sys:quantity', 'Mængde', 'Kortlægning'),
  key('lex:a', 'Producent', 'Produkt'),
  key('lex:b', 'Model', 'Produkt'),
  key('lex:c', 'Brandklasse', 'Brand'),
  key('col:1', 'Farve', 'Egne felter'),
];

describe('member identity', () => {
  it('names categories and keys apart', () => {
    expect(memberId({ category: 'Produkt' })).toBe('category:Produkt');
    expect(memberId({ key: 'lex:a' })).toBe('key:lex:a');
  });
});

describe('categories', () => {
  it('counts keys per category in catalogue order', () => {
    expect(categoryCounts(KEYS)).toEqual([
      { category: 'Kortlægning', count: 2 },
      { category: 'Produkt', count: 2 },
      { category: 'Brand', count: 1 },
      { category: 'Egne felter', count: 1 },
    ]);
  });

  it('toggles a category on at the end and off where it was', () => {
    const on = toggleCategory([{ key: 'sys:name' }], 'Produkt');
    expect(on).toEqual([{ key: 'sys:name' }, { category: 'Produkt' }]);
    expect(hasCategory(on, 'Produkt')).toBe(true);
    expect(toggleCategory(on, 'Produkt')).toEqual([{ key: 'sys:name' }]);
  });
});

describe('editing the list', () => {
  const M: TemplateMember[] = [{ key: 'sys:name' }, { category: 'Produkt' }, { key: 'col:1' }];

  it('adds a key once', () => {
    expect(addKey(M, 'lex:c')).toEqual([...M, { key: 'lex:c' }]);
    expect(addKey(M, 'sys:name')).toBe(M);
  });

  it('removes by position, including a missing member', () => {
    expect(removeMemberAt(M, 1)).toEqual([{ key: 'sys:name' }, { key: 'col:1' }]);
    expect(removeMemberAt(M, 9)).toBe(M);
  });

  it('moves a member and leaves out-of-range moves alone', () => {
    expect(moveMember(M, 2, 0)).toEqual([{ key: 'col:1' }, { key: 'sys:name' }, { category: 'Produkt' }]);
    expect(moveMember(M, 0, 1)).toEqual([{ category: 'Produkt' }, { key: 'sys:name' }, { key: 'col:1' }]);
    expect(moveMember(M, 0, -1)).toBe(M);
    expect(moveMember(M, 2, 3)).toBe(M);
    expect(moveMember(M, 1, 1)).toBe(M);
  });

  it('never mutates its input', () => {
    const copy = structuredClone(M);
    toggleCategory(M, 'Brand');
    addKey(M, 'lex:c');
    removeMemberAt(M, 0);
    moveMember(M, 0, 2);
    expect(M).toEqual(copy);
  });
});

describe('resolveMembers (spec §5.2)', () => {
  it('expands categories in catalogue order and keeps the first position of a duplicate', () => {
    const r = resolveMembers([{ key: 'lex:b' }, { category: 'Produkt' }, { key: 'sys:name' }], KEYS);
    expect(r.keys).toEqual(['lex:b', 'lex:a', 'sys:name']);
    expect(r.missing).toEqual([]);
  });

  it('skips and reports members that no longer exist', () => {
    const r = resolveMembers([{ key: 'col:99' }, { category: 'Fjernet' }, { key: 'col:1' }], KEYS);
    expect(r.keys).toEqual(['col:1']);
    expect(r.missing).toEqual([{ key: 'col:99' }, { category: 'Fjernet' }]);
  });

  it('picks up a key added to a category member later', () => {
    const more = [...KEYS, key('lex:d', 'Årgang', 'Produkt')];
    expect(resolveMembers([{ category: 'Produkt' }], more).keys).toEqual(['lex:a', 'lex:b', 'lex:d']);
  });

  it('resolves an empty template to nothing', () => {
    expect(resolveMembers([], KEYS)).toEqual({ keys: [], missing: [] });
  });
});

describe('searchKeys', () => {
  it('matches label, id and category case-insensitively, minus explicit key members', () => {
    expect(searchKeys(KEYS, 'PROD', [], 20).map((k) => k.id)).toEqual(['lex:a', 'lex:b']);
    expect(searchKeys(KEYS, 'col:', [], 20).map((k) => k.id)).toEqual(['col:1']);
    expect(searchKeys(KEYS, 'prod', [{ key: 'lex:a' }], 20).map((k) => k.id)).toEqual(['lex:b']);
  });

  it('returns nothing for a blank query and honours the limit', () => {
    expect(searchKeys(KEYS, '   ', [], 20)).toEqual([]);
    expect(searchKeys(KEYS, 'e', [], 2)).toHaveLength(2);
  });
});

describe('memberLabel', () => {
  it('labels a category with its key count and a key with its category', () => {
    expect(memberLabel({ category: 'Produkt' }, KEYS)).toEqual({ label: 'Produkt', kind: 'Kategori', detail: '2 felter' });
    expect(memberLabel({ key: 'lex:c' }, KEYS)).toEqual({ label: 'Brandklasse', kind: 'Felt', detail: 'Brand' });
  });

  it('falls back to the raw id for a missing member', () => {
    expect(memberLabel({ key: 'col:99' }, KEYS)).toEqual({ label: 'col:99', kind: 'Felt', detail: 'Findes ikke længere' });
    expect(memberLabel({ category: 'Fjernet' }, KEYS)).toEqual({ label: 'Fjernet', kind: 'Kategori', detail: 'Findes ikke længere' });
  });
});

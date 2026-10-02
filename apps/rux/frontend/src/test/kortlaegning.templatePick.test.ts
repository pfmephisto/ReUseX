// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  appendKeyMember,
  pickTemplate,
  readStoredTemplateId,
  templateStorageKey,
  writeStoredTemplateId,
  type StorageLike,
} from '../kortlaegning/templatePick';
import { template } from './resourceFixtures';

function memoryStorage(): StorageLike & { data: Map<string, string> } {
  const data = new Map<string, string>();
  return { data, getItem: (k) => data.get(k) ?? null, setItem: (k, v) => void data.set(k, v) };
}

const full = template({ id: 1, name: 'Materialepas (fuld)', seed: 'materialepas' });
const screening = template({ id: 2, name: 'Hurtig genbrugsscreening', seed: 'screening' });
const own = template({ id: 5, name: 'Min', seed: null });

describe('pickTemplate', () => {
  it('keeps the stored template while it exists', () => {
    expect(pickTemplate([full, screening, own], 5)).toBe(own);
  });

  it('falls back to the screening seed, then the first template, then nothing', () => {
    expect(pickTemplate([full, screening, own], 99)).toBe(screening);
    expect(pickTemplate([full, screening], null)).toBe(screening);
    expect(pickTemplate([own, full], null)).toBe(own);
    expect(pickTemplate([], 2)).toBeNull();
  });
});

describe('stored template per project', () => {
  it('round-trips under a per-project key', () => {
    const s = memoryStorage();
    writeStoredTemplateId('malov.rux', 5, s);
    expect(s.data.get(templateStorageKey('malov.rux'))).toBe('5');
    expect(readStoredTemplateId('malov.rux', s)).toBe(5);
    expect(readStoredTemplateId('other.rux', s)).toBeNull();
  });

  it('ignores junk and a storage that throws', () => {
    const s = memoryStorage();
    s.data.set(templateStorageKey('p'), 'abc');
    expect(readStoredTemplateId('p', s)).toBeNull();
    const broken: StorageLike = {
      getItem: () => {
        throw new Error('denied');
      },
      setItem: () => {
        throw new Error('denied');
      },
    };
    expect(readStoredTemplateId('p', broken)).toBeNull();
    expect(() => writeStoredTemplateId('p', 1, broken)).not.toThrow();
  });
});

describe('appendKeyMember', () => {
  it('appends a key member once', () => {
    const members = [{ category: 'Egne felter' }, { key: 'sys:name' }];
    expect(appendKeyMember(members, 'col:4')).toEqual([...members, { key: 'col:4' }]);
    expect(appendKeyMember([{ key: 'col:4' }], 'col:4')).toEqual([{ key: 'col:4' }]);
  });
});

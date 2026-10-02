// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  appendKeyMember,
  pickTemplate,
  projectIdentity,
  readStoredTemplateId,
  TEMPLATE_STORAGE_PREFIX,
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
  const malov = projectIdentity([{ id: 'p-malov', name: 'Malov' }], 'project.rux');
  const other = projectIdentity([{ id: 'p-other', name: 'Andet' }], 'project.rux');

  it('round-trips under a per-project key', () => {
    const s = memoryStorage();
    writeStoredTemplateId(malov, 5, s);
    expect(s.data.get(templateStorageKey(malov))).toBe('5');
    expect(readStoredTemplateId(malov, s)).toBe(5);
    expect(readStoredTemplateId(other, s)).toBeNull();
  });

  it('two projects with the same filename keep separate choices', () => {
    const s = memoryStorage();
    expect(templateStorageKey(malov)).not.toBe(templateStorageKey(other));
    writeStoredTemplateId(malov, 3, s);
    writeStoredTemplateId(other, 7, s);
    expect(readStoredTemplateId(malov, s)).toBe(3);
    expect(readStoredTemplateId(other, s)).toBe(7);
  });

  it('the same record id under two filenames is two projects', () => {
    const a = projectIdentity([{ id: 'default', name: 'X' }], 'a.rux');
    const b = projectIdentity([{ id: 'default', name: 'X' }], 'b.rux');
    expect(templateStorageKey(a)).not.toBe(templateStorageKey(b));
  });

  it('the identity is the project record id, else the filename', () => {
    expect(projectIdentity([{ id: 'abc', name: 'X' }], 'a.rux')).toEqual({ id: 'abc', name: 'a.rux' });
    expect(projectIdentity([], 'a.rux')).toEqual({ id: null, name: 'a.rux' });
    expect(templateStorageKey({ id: null, name: 'a.rux' })).toBe(`${TEMPLATE_STORAGE_PREFIX}a.rux`);
  });

  it('reads the old filename key once, then keeps the choice under the new key', () => {
    const s = memoryStorage();
    s.data.set(`${TEMPLATE_STORAGE_PREFIX}project.rux`, '4');
    expect(readStoredTemplateId(malov, s)).toBe(4);
    expect(s.data.get(templateStorageKey(malov))).toBe('4');
    // Later choices win over the old key.
    writeStoredTemplateId(malov, 9, s);
    expect(readStoredTemplateId(malov, s)).toBe(9);
  });

  it('ignores junk and a storage that throws', () => {
    const p = { id: null, name: 'p' };
    const s = memoryStorage();
    s.data.set(templateStorageKey(p), 'abc');
    expect(readStoredTemplateId(p, s)).toBeNull();
    const broken: StorageLike = {
      getItem: () => {
        throw new Error('denied');
      },
      setItem: () => {
        throw new Error('denied');
      },
    };
    expect(readStoredTemplateId(p, broken)).toBeNull();
    expect(readStoredTemplateId(malov, broken)).toBeNull();
    expect(() => writeStoredTemplateId(p, 1, broken)).not.toThrow();
  });
});

describe('appendKeyMember', () => {
  it('appends a key member once', () => {
    const members = [{ category: 'Egne felter' }, { key: 'sys:name' }];
    expect(appendKeyMember(members, 'col:4')).toEqual([...members, { key: 'col:4' }]);
    expect(appendKeyMember([{ key: 'col:4' }], 'col:4')).toEqual([{ key: 'col:4' }]);
  });
});

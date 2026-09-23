// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Theme resolution and persistence contract.
 *
 * The DOM only ever carries a *resolved* theme (`light` | `dark`); the
 * unresolved preference (`system` included) lives in localStorage. These
 * assertions pin that boundary — a `system` choice must defer to the OS, an
 * explicit choice must override it, and an unrecognised stored value must
 * degrade to `system` rather than throw. No DOM is needed, so the Node test
 * env exercises it directly with an injected storage stub.
 */

import { describe, expect, it } from 'vitest';
import {
  THEME_PREFERENCES,
  THEME_STORAGE_KEY,
  isThemePreference,
  readStoredPreference,
  resolveTheme,
  storePreference,
} from '../theme';

/** Minimal in-memory `Storage` — enough for read/store round-trips. */
function fakeStorage(initial: Record<string, string> = {}): Storage {
  const map = new Map<string, string>(Object.entries(initial));
  return {
    getItem: (key) => (map.has(key) ? map.get(key)! : null),
    setItem: (key, value) => void map.set(key, String(value)),
    removeItem: (key) => void map.delete(key),
    clear: () => map.clear(),
    key: (index) => Array.from(map.keys())[index] ?? null,
    get length() {
      return map.size;
    },
  } as Storage;
}

describe('resolveTheme', () => {
  it('defers to the OS when the preference is system', () => {
    expect(resolveTheme('system', true)).toBe('dark');
    expect(resolveTheme('system', false)).toBe('light');
  });

  it('forces the chosen theme regardless of the OS', () => {
    expect(resolveTheme('light', true)).toBe('light');
    expect(resolveTheme('light', false)).toBe('light');
    expect(resolveTheme('dark', true)).toBe('dark');
    expect(resolveTheme('dark', false)).toBe('dark');
  });
});

describe('isThemePreference', () => {
  it('accepts exactly the three preferences', () => {
    expect(isThemePreference('system')).toBe(true);
    expect(isThemePreference('light')).toBe(true);
    expect(isThemePreference('dark')).toBe(true);
  });

  it('rejects anything else', () => {
    for (const value of [null, undefined, '', 'auto', 'Light', 0, {}]) {
      expect(isThemePreference(value)).toBe(false);
    }
  });
});

describe('THEME_PREFERENCES', () => {
  it('lists the three choices with system (the default) first', () => {
    expect([...THEME_PREFERENCES]).toEqual(['system', 'light', 'dark']);
    expect(THEME_PREFERENCES[0]).toBe('system');
  });
});

describe('readStoredPreference / storePreference', () => {
  it('defaults to system when nothing is stored', () => {
    expect(readStoredPreference(fakeStorage())).toBe('system');
  });

  it('defaults to system when the stored value is not a preference', () => {
    expect(readStoredPreference(fakeStorage({ [THEME_STORAGE_KEY]: 'sepia' }))).toBe(
      'system',
    );
  });

  it('returns a valid stored preference', () => {
    expect(readStoredPreference(fakeStorage({ [THEME_STORAGE_KEY]: 'light' }))).toBe(
      'light',
    );
  });

  it('round-trips through storage under the shared key', () => {
    const storage = fakeStorage();
    storePreference('dark', storage);
    expect(storage.getItem(THEME_STORAGE_KEY)).toBe('dark');
    expect(readStoredPreference(storage)).toBe('dark');
  });
});

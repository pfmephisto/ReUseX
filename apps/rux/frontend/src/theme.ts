// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Colour-theme model: the user's *preference* versus the *resolved* theme.
 *
 * A preference of `system` defers to the OS (`prefers-color-scheme`); `light`
 * and `dark` force one. The DOM only ever carries a resolved value on
 * `<html data-theme>` — `light` or `dark` — because CSS has one light token
 * block to override, not two. The unresolved preference lives in localStorage.
 *
 * This module is deliberately DOM-free where it can be (pure `resolveTheme` /
 * `isThemePreference`) so the resolution contract is unit-testable in Node; the
 * storage and apply helpers guard their globals and accept injected handles.
 */

export type ThemePreference = 'system' | 'light' | 'dark';
export type ResolvedTheme = 'light' | 'dark';

/**
 * localStorage key holding the user's theme preference. The inline FOUC-guard
 * script in `index.html` reads this same key before first paint — keep the two
 * in sync.
 */
export const THEME_STORAGE_KEY = 'reusex-theme';

/** The three choices, in the order the toggle presents them (default first). */
export const THEME_PREFERENCES = ['system', 'light', 'dark'] as const;

export function isThemePreference(value: unknown): value is ThemePreference {
  return value === 'system' || value === 'light' || value === 'dark';
}

/**
 * Collapse a preference to the theme actually painted, given whether the OS
 * currently asks for dark. `system` follows the OS; the others force.
 */
export function resolveTheme(
  preference: ThemePreference,
  systemPrefersDark: boolean,
): ResolvedTheme {
  if (preference === 'system') return systemPrefersDark ? 'dark' : 'light';
  return preference;
}

/** localStorage, or `undefined` where it is absent or access throws (sandbox). */
function defaultStorage(): Storage | undefined {
  try {
    return typeof localStorage !== 'undefined' ? localStorage : undefined;
  } catch {
    return undefined;
  }
}

/** The stored preference, defaulting to `system` when missing or unrecognised. */
export function readStoredPreference(storage = defaultStorage()): ThemePreference {
  try {
    const raw = storage?.getItem(THEME_STORAGE_KEY);
    return isThemePreference(raw) ? raw : 'system';
  } catch {
    return 'system';
  }
}

export function storePreference(
  preference: ThemePreference,
  storage = defaultStorage(),
): void {
  try {
    storage?.setItem(THEME_STORAGE_KEY, preference);
  } catch {
    // Private mode / quota / denied — the choice simply will not persist.
  }
}

/** Paint the resolved theme by setting `data-theme` on the document root. */
export function applyTheme(
  resolved: ResolvedTheme,
  root: HTMLElement = document.documentElement,
): void {
  root.dataset.theme = resolved;
}

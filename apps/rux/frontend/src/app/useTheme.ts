// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useState } from 'react';

import type { ResolvedTheme, ThemePreference } from '../theme';
import {
  applyTheme,
  readStoredPreference,
  resolveTheme,
  storePreference,
} from '../theme';

const DARK_QUERY = '(prefers-color-scheme: dark)';

/** Whether the OS currently asks for dark; dark-first when it cannot be asked. */
function systemPrefersDark(): boolean {
  return typeof window !== 'undefined' && typeof window.matchMedia === 'function'
    ? window.matchMedia(DARK_QUERY).matches
    : true;
}

export interface UseThemeResult {
  /** The user's choice, including `system`. */
  preference: ThemePreference;
  /** What is actually painted right now — `light` or `dark`. */
  resolved: ResolvedTheme;
  setPreference: (preference: ThemePreference) => void;
}

/**
 * Own the theme: read the stored preference, follow the OS live while it is
 * `system`, paint the resolved value onto `<html data-theme>`, and persist any
 * change. The inline script in `index.html` has already applied the correct
 * theme before first paint; this hook keeps it in sync afterwards, so mounting
 * it re-applies the identical value (idempotent) rather than causing a flash.
 *
 * Mount exactly one instance (the shell's toggle) — a second would add a
 * redundant `matchMedia` listener.
 */
export function useTheme(): UseThemeResult {
  const [preference, setPreferenceState] = useState<ThemePreference>(() =>
    readStoredPreference(),
  );
  const [prefersDark, setPrefersDark] = useState<boolean>(() => systemPrefersDark());

  // Track the OS setting so a `system` preference follows it without a reload.
  useEffect(() => {
    if (typeof window === 'undefined' || typeof window.matchMedia !== 'function') {
      return;
    }
    const media = window.matchMedia(DARK_QUERY);
    const onChange = (event: MediaQueryListEvent) => setPrefersDark(event.matches);
    media.addEventListener('change', onChange);
    return () => media.removeEventListener('change', onChange);
  }, []);

  const resolved = resolveTheme(preference, prefersDark);

  useEffect(() => {
    applyTheme(resolved);
  }, [resolved]);

  const setPreference = useCallback((next: ThemePreference) => {
    setPreferenceState(next);
    storePreference(next);
  }, []);

  return { preference, resolved, setPreference };
}

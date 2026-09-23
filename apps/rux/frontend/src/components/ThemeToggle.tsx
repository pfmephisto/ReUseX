// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useTheme } from '../app/useTheme';
import { THEME_PREFERENCES } from '../theme';
import type { ThemePreference } from '../theme';
import styles from './ThemeToggle.module.css';

const LABELS: Record<ThemePreference, string> = {
  system: 'System',
  light: 'Light',
  dark: 'Dark',
};

/**
 * Icons drawn with `currentColor` and sized from a type token, so they inherit
 * the segment's text colour in both themes without any per-icon override.
 */
function ThemeIcon({ preference }: { preference: ThemePreference }) {
  const common = {
    className: styles.icon,
    viewBox: '0 0 24 24',
    fill: 'none',
    stroke: 'currentColor',
    strokeWidth: 2,
    strokeLinecap: 'round' as const,
    strokeLinejoin: 'round' as const,
    'aria-hidden': true,
  };
  switch (preference) {
    case 'light':
      return (
        <svg {...common}>
          <circle cx="12" cy="12" r="4" />
          <path d="M12 2v2M12 20v2M2 12h2M20 12h2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4" />
        </svg>
      );
    case 'dark':
      return (
        <svg {...common}>
          <path d="M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z" />
        </svg>
      );
    case 'system':
      return (
        <svg {...common}>
          <rect x="3" y="4" width="18" height="12" rx="1" />
          <path d="M8 20h8M12 16v4" />
        </svg>
      );
  }
}

/**
 * Three-way Light / Dark / System theme control for the title bar.
 *
 * A single-select radio group: the checked segment is the stored *preference*
 * (which may be `system`), not the resolved theme. Selecting one persists it
 * and repaints the whole app via the `data-theme` attribute — no per-component
 * work, since every style already reads `var(--…)` tokens.
 */
export function ThemeToggle() {
  const { preference, setPreference } = useTheme();

  return (
    <div className={styles.group} role="radiogroup" aria-label="Colour theme">
      {THEME_PREFERENCES.map((option) => {
        const active = option === preference;
        return (
          <button
            key={option}
            type="button"
            role="radio"
            aria-checked={active}
            className={`${styles.option} ${active ? styles.active : ''}`}
            title={`${LABELS[option]} theme`}
            onClick={() => setPreference(option)}
          >
            <ThemeIcon preference={option} />
            <span className={styles.srOnly}>{LABELS[option]} theme</span>
          </button>
        );
      })}
    </div>
  );
}

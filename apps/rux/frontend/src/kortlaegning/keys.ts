// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's keyboard flow as a pure key → action map, one for the table and
 * one for the edit dialog. Kept out of the components so "never fire while the
 * user is typing" is a tested rule, not an onKeyDown detail.
 */

/** The evidence views, in tab order: Plan · 360° · Foto · Punktsky · Rum (keys 1–5). */
export type EvidenceTab = 'plan' | 'pano' | 'foto' | 'punktsky' | 'rum';
export const EVIDENCE_TABS: readonly EvidenceTab[] = ['plan', 'pano', 'foto', 'punktsky', 'rum'];

/** The last evidence number key, for the hint text (`1`–`5`). */
export const EVIDENCE_LAST_KEY = String(EVIDENCE_TABS.length);

export type KortAction =
  | { type: 'move'; delta: number }
  | { type: 'expand' }
  | { type: 'collapse' }
  | { type: 'open' }
  | { type: 'approve' }
  | { type: 'approveNext' }
  | { type: 'reject' }
  | { type: 'star' }
  | { type: 'evidence'; tab: EvidenceTab }
  | { type: 'blur' }
  | { type: 'close' };

export interface KeyInput {
  key: string;
  metaKey?: boolean;
  ctrlKey?: boolean;
  altKey?: boolean;
  /** True when focus is in an input, select or textarea. */
  inField: boolean;
  /**
   * True when focus is on a button or link (a tab, a chevron): Enter and
   * Space then belong to its native activation, not to the table.
   */
  isControl?: boolean;
}

function letterAction(key: string): KortAction | null {
  switch (key.toLowerCase()) {
    case 'g':
      return { type: 'approve' };
    case 'a':
      return { type: 'reject' };
    case 'v':
      return { type: 'star' };
    default:
      break;
  }
  const n = Number(key);
  if (Number.isInteger(n) && n >= 1 && n <= EVIDENCE_TABS.length) {
    return { type: 'evidence', tab: EVIDENCE_TABS[n - 1] };
  }
  return null;
}

export function tableAction(k: KeyInput): KortAction | null {
  if (k.key === 'Escape') return k.inField ? { type: 'blur' } : null;
  if (k.inField || k.metaKey || k.ctrlKey || k.altKey) return null;
  if (k.isControl && (k.key === 'Enter' || k.key === ' ')) return null;
  switch (k.key) {
    case 'ArrowDown':
    case 'j':
      return { type: 'move', delta: 1 };
    case 'ArrowUp':
    case 'k':
      return { type: 'move', delta: -1 };
    case 'ArrowRight':
      return { type: 'expand' };
    case 'ArrowLeft':
      return { type: 'collapse' };
    case 'Enter':
      return { type: 'open' };
    default:
      return letterAction(k.key);
  }
}

export function dialogAction(k: KeyInput): KortAction | null {
  if (k.key === 'Escape') return { type: 'close' };
  if (k.key === 'Enter' && (k.metaKey || k.ctrlKey)) return { type: 'approveNext' };
  if (k.key === 'PageDown') return { type: 'move', delta: 1 };
  if (k.key === 'PageUp') return { type: 'move', delta: -1 };
  if (k.inField || k.metaKey || k.ctrlKey || k.altKey) return null;
  return letterAction(k.key);
}

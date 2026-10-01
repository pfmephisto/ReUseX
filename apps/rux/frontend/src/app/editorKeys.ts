// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Keys inside an in-place editor (Miljø's sample editor and create form,
 * Overblik's case details). Shared so every screen follows R10: Esc in a text
 * field reverts it without committing; Esc anywhere else closes.
 */

import type { TargetKind } from './keyTargets';

export type EditorKey = 'revert' | 'close' | 'commit' | 'submit';

/**
 * Esc in a text field drops that field's draft (and must not commit it); Esc
 * anywhere else closes. Enter in a single-line text field commits it;
 * Ctrl/⌘+Enter submits from anywhere. Enter/Space on buttons, links and
 * checkboxes stay native.
 */
export function editorKeyAction(k: {
  key: string;
  kind: TargetKind;
  ctrlKey?: boolean;
  metaKey?: boolean;
  altKey?: boolean;
}): EditorKey | null {
  if (k.key === 'Escape') return k.kind === 'text' ? 'revert' : 'close';
  if (k.key === 'Enter' && (k.ctrlKey || k.metaKey)) return 'submit';
  if (k.key === 'Enter' && k.kind === 'text' && !k.altKey) return 'commit';
  return null;
}

/**
 * What a key inside a committing text field does itself: `revert` (Esc) or
 * `commit` (Enter, by leaving the field). In a multi-line field Enter is a
 * new line, never a commit. Everything else — `submit` included — is left to
 * bubble to the editor.
 */
export function fieldKeyAction(
  k: { key: string; ctrlKey?: boolean; metaKey?: boolean; altKey?: boolean },
  multiline = false,
): 'revert' | 'commit' | null {
  const action = editorKeyAction({ ...k, kind: 'text' });
  if (action === 'revert') return 'revert';
  if (action === 'commit' && !multiline) return 'commit';
  return null;
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What a key event landed on, so page shortcuts never fire while the user is
 * typing and never steal Enter/Space from a button, link or checkbox.
 * Classifies by tag name and input type only, so it is testable in Node.
 */

export type TargetKind = 'text' | 'choice' | 'control' | 'other';

export interface TargetLike {
  tagName: string;
  type?: string;
}

const CHOICE_INPUTS = new Set(['checkbox', 'radio']);
const CONTROL_INPUTS = new Set(['button', 'submit', 'reset']);

export function targetKind(t: TargetLike | null | undefined): TargetKind {
  if (!t || typeof t.tagName !== 'string') return 'other';
  const tag = t.tagName.toUpperCase();
  if (tag === 'TEXTAREA') return 'text';
  if (tag === 'SELECT') return 'choice';
  if (tag === 'INPUT') {
    const type = (t.type ?? 'text').toLowerCase();
    if (CHOICE_INPUTS.has(type)) return 'choice';
    if (CONTROL_INPUTS.has(type)) return 'control';
    return 'text';
  }
  if (tag === 'BUTTON' || tag === 'A') return 'control';
  return 'other';
}

export function kindOf(target: EventTarget | null): TargetKind {
  return target !== null && typeof target === 'object' && 'tagName' in target
    ? targetKind(target as unknown as TargetLike)
    : 'other';
}

/** Typing or choosing targets: inputs, selects, textareas. */
export function isField(target: EventTarget | null): boolean {
  const k = kindOf(target);
  return k === 'text' || k === 'choice';
}

/** Buttons and links: Enter/Space belong to their native activation. */
export function isControl(target: EventTarget | null): boolean {
  return kindOf(target) === 'control';
}

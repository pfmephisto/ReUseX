// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A modal's Tab trap, shared by Kortlægning's EditDialog and FormDialog: Tab
 * past the last focusable wraps to the first, Shift+Tab before the first to
 * the last. The index arithmetic and the tabbable test are pure, so they are
 * unit-tested without a DOM; `trapTab` wires them to a key event.
 */

import type { KeyboardEvent } from 'react';

/**
 * The Tab trap: given the focusable count, the index of the focused element
 * (`-1` when focus is outside the list, e.g. on the dialog root) and the Tab
 * direction, returns the index to move focus to — or `null` to let the
 * browser's own Tab order proceed (anywhere strictly inside the list). From
 * outside, Tab enters at `entry` (the quantity field) and Shift+Tab at the end.
 */
export function wrapFocusIndex(count: number, current: number, shift: boolean, entry = 0): number | null {
  if (count === 0) return null;
  if (current < 0) return shift ? count - 1 : entry >= 0 && entry < count ? entry : 0;
  if (shift && current === 0) return count - 1;
  if (!shift && current === count - 1) return 0;
  return null;
}

export const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), ' +
  'textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

/** The bits of an element `isTabbable` looks at; a structural type so tests can stub it. */
export interface TabbableProbe {
  /** `HTMLElement.hidden` is `boolean | "until-found"`; either truthy form hides. */
  hidden: boolean | string;
  getClientRects(): { length: number };
  closest(selectors: string): unknown;
}

/**
 * Whether a `FOCUSABLE` match can actually take Tab focus: not `hidden`,
 * rendered (has a layout box — `display: none` ancestors give none), and not
 * inside an `inert` subtree or a disabled fieldset.
 */
export function isTabbable(el: TabbableProbe): boolean {
  return !el.hidden && el.getClientRects().length > 0 && !el.closest('[inert],fieldset[disabled]');
}

/**
 * Keeps Tab inside `root`: moves focus and prevents the default when Tab
 * would leave it; otherwise lets the browser's own order proceed. `entry` is
 * where Tab from outside the list (e.g. from the root itself) lands.
 */
export function trapTab(e: KeyboardEvent<HTMLElement>, root: HTMLElement | null, entry: HTMLElement | null = null) {
  if (e.key !== 'Tab' || !root) return;
  const focusables = Array.from(root.querySelectorAll<HTMLElement>(FOCUSABLE)).filter(isTabbable);
  const index = focusables.indexOf(document.activeElement as HTMLElement);
  const next = wrapFocusIndex(focusables.length, index, e.shiftKey, entry ? focusables.indexOf(entry) : 0);
  if (next === null) return;
  e.preventDefault();
  const target = focusables[next];
  target.focus();
  // Native Tab selects a text field's contents; do the same, so typing replaces the value.
  if (target instanceof HTMLInputElement) target.select();
}

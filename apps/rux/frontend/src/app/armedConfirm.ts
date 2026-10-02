// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * When an armed two-click confirm ("Slet" → "Bekræft: slet") disarms. Pure,
 * so the rules are testable without a DOM; `useArmedConfirm` is the caller.
 * Blur alone is not enough: Safari and macOS Firefox never focus a clicked
 * button, and a busy flip disables it without a blur, so a stale armed
 * button could otherwise delete with a single click later.
 */
export type ArmedEvent =
  | { kind: 'key'; key: string }
  | { kind: 'pointerdown'; inside: boolean }
  | { kind: 'busy'; busy: boolean }
  | { kind: 'blur' };

export function disarms(e: ArmedEvent): boolean {
  switch (e.kind) {
    case 'key':
      return e.key === 'Escape';
    case 'pointerdown':
      return !e.inside;
    case 'busy':
      return e.busy;
    case 'blur':
      return true;
  }
}

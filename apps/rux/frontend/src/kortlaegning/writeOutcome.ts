// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What Kortlægning tells the user after a write. Pure, so the copy is tested
 * without a DOM.
 *
 * - A refused value (a 400 from `PATCH /resources/{code}`) names the key by
 *   its catalogue label. The server's own text carries the opaque key id
 *   (`'lex:294Zk…' must be a whole number`) and is English, so it never
 *   reaches the user.
 * - A write that succeeded is never reported as failed because the re-read
 *   after it failed: that gets its own notice, and the save stands.
 */

import { ApiRequestError } from '../api/client';
import { saveErrorMessage } from '../app/saveError';

/** The notice after a successful write whose follow-up re-read failed. */
export const REFRESH_FAILED = 'Visningen kunne ikke opdateres — genindlæs siden';

/** The copy for a value the client could not parse or the server refused. */
export function invalidValueMessage(label: string): string {
  return `Ugyldig værdi for ${label} — ikke gemt`;
}

/** The toast for a failed resource-value write. */
export function cellErrorMessage(cause: unknown, label: string): string {
  if (cause instanceof ApiRequestError && cause.status === 400) return invalidValueMessage(label);
  return saveErrorMessage(cause);
}

/**
 * Runs the re-read after a successful write. A failure calls `onFailed` with
 * `REFRESH_FAILED` and resolves false; it never throws, so the caller's
 * success path (toast, closing a dialog) is not undone.
 */
export async function refreshAfterSave(
  refresh: () => Promise<unknown>,
  onFailed: (message: string) => void,
): Promise<boolean> {
  try {
    await refresh();
    return true;
  } catch {
    onFailed(REFRESH_FAILED);
    return false;
  }
}

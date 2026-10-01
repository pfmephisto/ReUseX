// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The app's one write chain (GUI Phase 6, R11). Every screen's writes join
 * it through `useMutationQueue`, and the case screens start their first load
 * with `appWriteChain.idle()`. A write queued on the screen just left — a
 * sample registered on the phone sheet, a result recorded in Miljø & prøver —
 * therefore lands before the next screen reads the survey, so that screen
 * never shows the approval gate as it was before the write.
 *
 * Rapport opts out (`scope: 'page'`): report generation can take minutes,
 * changes no survey state, and must not hold up other screens' first loads.
 */

import { createSerialQueue, type SerialQueue } from './serialQueue';

export const appWriteChain: SerialQueue = createSerialQueue();

/** The chain a page's writes join: `appWriteChain`, or a fresh one for 'page'. */
export function chainFor(scope: 'app' | 'page' = 'app'): SerialQueue {
  return scope === 'page' ? createSerialQueue() : appWriteChain;
}

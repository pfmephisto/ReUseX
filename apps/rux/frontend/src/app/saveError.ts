// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { ApiRequestError } from '../api/client';

export function errorMessage(cause: unknown): string {
  return cause instanceof Error ? cause.message : String(cause);
}

/**
 * The toast for a failed save. A running pipeline job (409) and a server that
 * is not ready (503) are transient and get their own Danish copy; anything
 * else shows the server's message.
 */
export function saveErrorMessage(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 409) return 'Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.';
    if (cause.status === 503) return 'Kunne ikke gemme — serveren er ikke klar.';
  }
  return `Kunne ikke gemme: ${errorMessage(cause)}`;
}

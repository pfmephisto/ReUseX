// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Skabeloner as data (spec §6.2): the copy, how errors read, the new-name and
 * selection rules, list-state replacement and the gate that keeps a stale
 * save-on-change response from overwriting a newer edit (R3).
 */

import { ApiRequestError } from '../api/client';
import type { Template, TemplateMember } from '../api/types';
import { errorMessage, saveErrorMessage } from '../app/saveError';

export const SEED_TAGS = ['materialepas', 'screening'] as const;

const SEED_NAMES: Record<string, string> = {
  materialepas: 'Materialepas (fuld)',
  screening: 'Hurtig genbrugsscreening',
};

export function countLine(n: number): string {
  return n === 1 ? '1 felt' : `${n} felter`;
}

/** What a screen reader hears after a keyboard reorder (`to` is 0-based). */
export function moveAnnouncement(label: string, to: number, total: number): string {
  return `«${label}» flyttet til plads ${to + 1} af ${total}`;
}

export function seedLabel(seed: string | null): string | null {
  return seed ? 'Standard' : null;
}

export function nextTemplateName(names: readonly string[], base = 'Ny skabelon'): string {
  const taken = new Set(names);
  if (!taken.has(base)) return base;
  for (let i = 2; ; i += 1) if (!taken.has(`${base} ${i}`)) return `${base} ${i}`;
}

export function missingSeeds(templates: readonly Pick<Template, 'seed'>[]): string[] {
  const present = new Set(templates.map((t) => t.seed));
  return SEED_TAGS.filter((s) => !present.has(s));
}

export function restoreSeedsTitle(missing: readonly string[]): string {
  if (missing.length === 0) return 'Begge standardskabeloner findes i projektet.';
  return `Genopretter ${missing.map((s) => SEED_NAMES[s] ?? s).join(' og ')}.`;
}

export function deleteConfirmText(t: Pick<Template, 'name' | 'seed'>): string {
  return t.seed
    ? `Slet skabelonen "${t.name}"? Den kan hentes tilbage med "Gendan standardskabeloner".`
    : `Slet skabelonen "${t.name}"? Det kan ikke fortrydes.`;
}

/** 409 from the templates API (duplicate name), not from `with_write`'s job lock (R5). */
export function isNameConflict(cause: unknown): boolean {
  return cause instanceof ApiRequestError && cause.status === 409 && !cause.message.includes('pipeline job');
}

export function templateErrorMessage(cause: unknown): string {
  if (isNameConflict(cause)) return 'Der findes allerede en skabelon med det navn.';
  if (cause instanceof ApiRequestError) {
    if (cause.status === 404) return 'Skabelonen findes ikke længere — listen er hentet igen.';
    if (cause.status === 400) return `Ugyldig skabelon: ${errorMessage(cause)}`;
  }
  return saveErrorMessage(cause);
}

export function selectAfterDelete(ids: readonly number[], deletedId: number): number | null {
  const i = ids.indexOf(deletedId);
  if (i === -1) return ids[0] ?? null;
  return ids[i + 1] ?? ids[i - 1] ?? null;
}

export function replaceTemplate<T extends { id: number }>(list: readonly T[], next: T): T[] {
  return list.map((t) => (t.id === next.id ? next : t));
}

export function withMembers<T extends { id: number; members: TemplateMember[] }>(
  list: readonly T[],
  id: number,
  members: TemplateMember[],
): T[] {
  return list.map((t) => (t.id === id ? { ...t, members } : t));
}

export function createLatestGate(): { next(id: number): number; isLatest(id: number, ticket: number): boolean } {
  const latest = new Map<number, number>();
  return {
    next(id) {
      const n = (latest.get(id) ?? 0) + 1;
      latest.set(id, n);
      return n;
    },
    isLatest(id, ticket) {
      return latest.get(id) === ticket;
    },
  };
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Which template Kortlægning shows (spec §6.1): the one remembered for this
 * project, else the `screening` seed, else the first. Remembered in
 * localStorage per project; storage that is missing or throws (sandbox,
 * privacy mode) is treated as empty, never as an error.
 */

import type { Template, TemplateMember } from '../api/types';

export const TEMPLATE_STORAGE_PREFIX = 'rux.kortlaegning.template:';

export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

function defaultStorage(): StorageLike | undefined {
  try {
    return typeof localStorage !== 'undefined' ? localStorage : undefined;
  } catch {
    return undefined;
  }
}

export function templateStorageKey(project: string): string {
  return `${TEMPLATE_STORAGE_PREFIX}${project}`;
}

export function readStoredTemplateId(project: string, storage = defaultStorage()): number | null {
  try {
    const raw = storage?.getItem(templateStorageKey(project));
    if (raw === null || raw === undefined || !/^\d+$/.test(raw)) return null;
    return Number(raw);
  } catch {
    return null;
  }
}

export function writeStoredTemplateId(project: string, id: number, storage = defaultStorage()): void {
  try {
    storage?.setItem(templateStorageKey(project), String(id));
  } catch {
    // A refused write only means the choice is not remembered.
  }
}

export function pickTemplate(templates: readonly Template[], storedId: number | null): Template | null {
  return (
    templates.find((t) => t.id === storedId) ??
    templates.find((t) => t.seed === 'screening') ??
    templates[0] ??
    null
  );
}

/** `members` plus a key member for `keyId`, unless one is already there. */
export function appendKeyMember(members: readonly TemplateMember[], keyId: string): TemplateMember[] {
  if (members.some((m) => 'key' in m && m.key === keyId)) return [...members];
  return [...members, { key: keyId }];
}

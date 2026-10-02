// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Which template Kortlægning shows (spec §6.1): the one remembered for this
 * project, else the `screening` seed, else the first. Remembered in
 * localStorage per project (`ProjectIdentity`); storage that is missing or throws (sandbox,
 * privacy mode) is treated as empty, never as an error.
 */

import type { ProjectInfo, Template, TemplateMember } from '../api/types';

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

/**
 * Which project a remembered choice belongs to. The filename alone is not
 * enough — every default `project.rux` shares it, and template ids are small
 * integers present in every project — so the project record's id (from
 * `GET /projects`) leads; the filename is only the fallback for a project
 * with no record yet.
 */
export interface ProjectIdentity {
  id: string | null;
  /** The project file's name (`GET /health` → `project.name`). */
  name: string;
}

export function projectIdentity(projects: readonly ProjectInfo[], name: string): ProjectIdentity {
  return { id: projects[0]?.id ?? null, name };
}

/** The key before project ids: the bare filename. */
function legacyStorageKey(project: ProjectIdentity): string {
  return `${TEMPLATE_STORAGE_PREFIX}${project.name}`;
}

export function templateStorageKey(project: ProjectIdentity): string {
  // The id and the filename together: a hand-set id such as `default`
  // (tests/fixtures/scans/README.md) can recur across projects too.
  return project.id === null
    ? legacyStorageKey(project)
    : `${TEMPLATE_STORAGE_PREFIX}id:${project.id}@${project.name}`;
}

function parseId(raw: string | null | undefined): number | null {
  return raw === null || raw === undefined || !/^\d+$/.test(raw) ? null : Number(raw);
}

/**
 * The remembered template id. With no entry under the project's own key, the
 * old filename key is read once and its value carried over to the new key, so
 * later reads (and writes) no longer depend on it.
 */
export function readStoredTemplateId(project: ProjectIdentity, storage = defaultStorage()): number | null {
  try {
    const key = templateStorageKey(project);
    const own = storage?.getItem(key);
    if (own !== null && own !== undefined) return parseId(own);
    if (project.id === null) return null;
    const legacy = parseId(storage?.getItem(legacyStorageKey(project)));
    if (legacy !== null) storage?.setItem(key, String(legacy));
    return legacy;
  } catch {
    return null;
  }
}

export function writeStoredTemplateId(project: ProjectIdentity, id: number, storage = defaultStorage()): void {
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

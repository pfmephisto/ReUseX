// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Cases in the URL (ruxd multi-case spec, phase S2), as data.
 *
 * Every case screen lives under `/sager/:cid/…`; `/sager` itself is the case
 * list. The case app runs in a router whose `basename` is the case prefix, so
 * every in-case path (`/kortlaegning`, `/viewport?pano=3`, …) keeps its old,
 * case-relative spelling, and a link inside a case can never leave it by
 * accident. Moving between cases, or back to the list, is a page load: each
 * case gets a fresh API client, a fresh events socket and a fresh viewport,
 * so nothing from one case can leak into another.
 *
 * An old unprefixed path (a bookmark from before cases) is sent on to the
 * same place in the last-used case, or to the list when there is none.
 *
 * Kept free of React so it is unit-tested in Node.
 */

import type { StorageLike } from './navigation';

/** The case list. */
export const CASES_PATH = '/sager';

/** localStorage key of the last case opened, for old unprefixed paths. */
export const LAST_CASE_KEY = 'rux.lastCase';

/** A case id as the server makes them: lower-case letters, digits, dashes. */
const CASE_ID = /^[a-z0-9][a-z0-9-]{0,63}$/;

export function isCaseId(value: string): boolean {
  return CASE_ID.test(value);
}

/** Where a pathname points. */
export type CaseLocation =
  | { kind: 'list' }
  | { kind: 'case'; cid: string; /** The in-case path, `/` for Overblik. */ rest: string }
  | { kind: 'legacy' };

export function parseCaseLocation(pathname: string): CaseLocation {
  if (pathname === CASES_PATH || pathname === `${CASES_PATH}/`) return { kind: 'list' };
  if (!pathname.startsWith(`${CASES_PATH}/`)) return { kind: 'legacy' };
  const after = pathname.slice(CASES_PATH.length + 1);
  const slash = after.indexOf('/');
  const raw = slash < 0 ? after : after.slice(0, slash);
  let cid: string;
  try {
    cid = decodeURIComponent(raw);
  } catch {
    return { kind: 'list' };
  }
  // Anything that is not an id the server could have made goes to the list.
  if (!isCaseId(cid)) return { kind: 'list' };
  const rest = slash < 0 ? '/' : after.slice(slash) || '/';
  return { kind: 'case', cid, rest };
}

/** The router basename of case `cid`. */
export function caseBasename(cid: string): string {
  return `${CASES_PATH}/${encodeURIComponent(cid)}`;
}

/** An absolute href into case `cid`; `rest` is an in-case path. */
export function caseHref(cid: string, rest = '/'): string {
  const base = caseBasename(cid);
  return rest === '/' || rest === '' ? base : `${base}${rest.startsWith('/') ? rest : `/${rest}`}`;
}

/**
 * Where an old unprefixed path goes: the same path in the last-used case, if
 * the server still has it, else the case list. The query is kept.
 */
export function legacyRedirectTarget(
  pathname: string,
  search: string,
  lastCase: string | null,
  caseIds: readonly string[],
): string {
  if (lastCase && caseIds.includes(lastCase)) return `${caseHref(lastCase, pathname)}${search}`;
  return CASES_PATH;
}

function defaultStorage(): StorageLike | undefined {
  try {
    return typeof localStorage !== 'undefined' ? localStorage : undefined;
  } catch {
    return undefined;
  }
}

/** The last case opened, or null; missing or throwing storage is none. */
export function readLastCase(storage = defaultStorage()): string | null {
  try {
    const raw = storage?.getItem(LAST_CASE_KEY) ?? null;
    return raw !== null && isCaseId(raw) ? raw : null;
  } catch {
    return null;
  }
}

/** Remembers the case; a storage failure only loses the memory. */
export function writeLastCase(cid: string, storage = defaultStorage()): void {
  try {
    storage?.setItem(LAST_CASE_KEY, cid);
  } catch {
    // Private mode or blocked site data: old links just land on the list.
  }
}

/** One chunk of an upload: bytes [start, end). */
export interface UploadChunk {
  start: number;
  end: number;
}

/**
 * The chunks a file of `size` bytes goes up in, resuming after `received`
 * bytes the server already holds. `chunkBytes` is the server's limit.
 */
export function uploadChunks(size: number, chunkBytes: number, received = 0): UploadChunk[] {
  if (!(chunkBytes > 0)) throw new RangeError('chunkBytes must be positive');
  const chunks: UploadChunk[] = [];
  for (let start = Math.max(0, received); start < size; start += chunkBytes) {
    chunks.push({ start, end: Math.min(size, start + chunkBytes) });
  }
  return chunks;
}

/** The case name an uploaded file suggests: its name without `.rux`. */
export function nameFromFile(fileName: string): string {
  return fileName.replace(/\.rux$/i, '').trim();
}

/** A byte count in Danish units: "1,2 GB", "640 kB". */
export function formatBytes(bytes: number): string {
  if (!Number.isFinite(bytes) || bytes < 0) return '—';
  const units = ['B', 'kB', 'MB', 'GB', 'TB'];
  let value = bytes;
  let unit = 0;
  while (value >= 1000 && unit < units.length - 1) {
    value /= 1000;
    unit += 1;
  }
  const digits = unit === 0 || value >= 100 ? 0 : 1;
  return `${value.toLocaleString('da-DK', { maximumFractionDigits: digits, minimumFractionDigits: digits })} ${units[unit]}`;
}

/**
 * What the case app does once the case's `GET /health` answers: remember the
 * case as last used only once the server has confirmed it (`remember`), and
 * leave for the list when the server does not have it (`leave`, a 404 — a
 * deleted case, or an old bookmark). Anything else (the server unreachable,
 * a 5xx) keeps the user where they are, so a restarting server does not throw
 * them out of their case.
 */
export function caseBootAction(outcome: { ok: true } | { ok: false; status?: number }): 'remember' | 'leave' | 'stay' {
  if (outcome.ok) return 'remember';
  return outcome.status === 404 ? 'leave' : 'stay';
}

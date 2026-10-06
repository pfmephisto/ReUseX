// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Kortlægning workbench as data: which types a tab and filters show, the
 * rows the table draws, how the selection moves, and how server responses are
 * folded back in. Pure, so the review flow is testable without a DOM.
 */

import type { EnvironmentStatus, SurveyPart, SurveyType } from '../api/types';

/** `all` is queue + approved; rejected types live only in `rejected` (Afvist). */
export type Tab = 'queue' | 'approved' | 'all' | 'rejected';
export type EnvFilter = 'ren' | 'afventer' | 'forurenet';
export interface Filters {
  search: string;
  roomId: number | null;
  env: EnvFilter | null;
  /** Only types that are ★, or have a ★ part (Phase 6 R13). */
  starred: boolean;
}
export const NO_FILTERS: Filters = { search: '', roomId: null, env: null, starred: false };

export type Selection = { typeId: number; partCode: string | null } | null;
export type Row =
  | { kind: 'type'; typeId: number }
  | { kind: 'part'; typeId: number; partCode: string };

export function envFilterOf(s: EnvironmentStatus): EnvFilter {
  if (s === 'afventer') return 'afventer';
  if (s === 'forurenet') return 'forurenet';
  return 'ren';
}

export function tabCounts(types: SurveyType[]): Record<Tab, number> {
  const queue = types.filter((t) => t.review_status === 'queue').length;
  const approved = types.filter((t) => t.review_status === 'approved').length;
  const rejected = types.filter((t) => t.review_status === 'rejected').length;
  return { queue, approved, all: queue + approved, rejected };
}

export function inTab(t: SurveyType, tab: Tab): boolean {
  if (tab === 'rejected') return t.review_status === 'rejected';
  if (t.review_status === 'rejected') return false;
  if (tab === 'queue') return t.review_status === 'queue';
  if (tab === 'approved') return t.review_status === 'approved';
  return true;
}

export function visibleTypes(types: SurveyType[], tab: Tab, f: Filters): SurveyType[] {
  const q = f.search.trim().toLowerCase();
  return types.filter(
    (t) =>
      inTab(t, tab) &&
      (q === '' || t.name.toLowerCase().includes(q)) &&
      (f.roomId === null || t.parts.some((p) => p.room_id === f.roomId)) &&
      (f.env === null || envFilterOf(t.environment_status) === f.env) &&
      (!f.starred || t.starred || t.parts.some((p) => p.starred)),
  );
}

export function flattenRows(types: SurveyType[], open: ReadonlySet<number>): Row[] {
  const rows: Row[] = [];
  for (const t of types) {
    rows.push({ kind: 'type', typeId: t.id });
    if (open.has(t.id)) {
      for (const p of [...t.parts].sort((a, b) => a.code.localeCompare(b.code))) {
        rows.push({ kind: 'part', typeId: t.id, partCode: p.code });
      }
    }
  }
  return rows;
}

export function sameSelection(row: Row, sel: Selection): boolean {
  if (!sel || row.typeId !== sel.typeId) return false;
  return row.kind === 'type' ? sel.partCode === null : row.partCode === sel.partCode;
}

function toSelection(row: Row): Selection {
  return { typeId: row.typeId, partCode: row.kind === 'part' ? row.partCode : null };
}

export function moveSelection(rows: Row[], sel: Selection, delta: number): Selection {
  if (rows.length === 0) return null;
  const at = rows.findIndex((r) => sameSelection(r, sel));
  if (at < 0) return toSelection(rows[0]);
  const next = Math.min(rows.length - 1, Math.max(0, at + delta));
  return toSelection(rows[next]);
}

/** A part's room as shown: its name, `Rum {id}` when unnamed, '' when it has no room. */
export function roomName(p: SurveyPart): string {
  if (p.room_name) return p.room_name;
  return p.room_id !== null ? `Rum ${p.room_id}` : '';
}

/** `RX-### · {room}`, or just the code when the part has no room. */
export function partLabel(p: SurveyPart): string {
  const room = roomName(p);
  return room ? `${p.code} · ${room}` : p.code;
}

export function roomOptions(types: SurveyType[]): { id: number; name: string }[] {
  const byId = new Map<number, string>();
  for (const t of types)
    for (const p of t.parts)
      if (p.room_id !== null && !byId.has(p.room_id)) byId.set(p.room_id, roomName(p));
  return [...byId].map(([id, name]) => ({ id, name })).sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

/**
 * The queued type to review after `afterTypeId`: the first one following it
 * (wrapping) that is not `afventer`, else the first `afventer` one, else null.
 * Order comes from `types`; `among` (default: all of them) limits the
 * candidates — the page passes what its tab and filters show, so the next
 * type is never one the surveyor has filtered out. `afterTypeId` need not be
 * in `among`: an approved type has just left the queue tab.
 */
export function nextInQueue(
  types: SurveyType[],
  afterTypeId: number,
  among: SurveyType[] = types,
): Selection {
  const eligible = new Set(among.filter((t) => t.review_status === 'queue').map((t) => t.id));
  if (eligible.size === 0) return null;
  const start = types.findIndex((t) => t.id === afterTypeId);
  const ordered = [...types.slice(start + 1), ...types.slice(0, start + 1)].filter((t) => eligible.has(t.id));
  const pick = ordered.find((t) => t.environment_status !== 'afventer') ?? ordered[0];
  return pick ? { typeId: pick.id, partCode: null } : null;
}

export function typeOf(types: SurveyType[], sel: Selection): SurveyType | null {
  return sel ? (types.find((t) => t.id === sel.typeId) ?? null) : null;
}

export function partOf(types: SurveyType[], sel: Selection): SurveyPart | null {
  if (!sel || sel.partCode === null) return null;
  return typeOf(types, sel)?.parts.find((p) => p.code === sel.partCode) ?? null;
}

export function replaceType(types: SurveyType[], updated: SurveyType): SurveyType[] {
  return types.map((t) => (t.id === updated.id ? updated : t));
}

export function replacePart(types: SurveyType[], updated: SurveyPart): SurveyType[] {
  return types.map((t) => {
    // A part re-filed to another type leaves this one and joins that one.
    const parts = t.parts.filter((p) => p.code !== updated.code);
    if (t.id === updated.type_id) parts.push(updated);
    if (parts.length === t.parts.length && t.id !== updated.type_id) return t;
    parts.sort((a, b) => a.code.localeCompare(b.code));
    return { ...t, parts, quantity: parts.reduce((s, p) => s + p.quantity, 0) };
  });
}

/** A deleted type leaves the list (its parts go with it). */
export function removeType(types: SurveyType[], typeId: number): SurveyType[] {
  return types.filter((t) => t.id !== typeId);
}

/** A deleted part leaves its type, whose quantity is re-summed. */
export function removePart(types: SurveyType[], code: string): SurveyType[] {
  return types.map((t) => {
    if (!t.parts.some((p) => p.code === code)) return t;
    const parts = t.parts.filter((p) => p.code !== code);
    return { ...t, parts, quantity: parts.reduce((s, p) => s + p.quantity, 0) };
  });
}

const TAB_OF: Record<SurveyType['review_status'], Tab> = {
  queue: 'queue',
  approved: 'approved',
  rejected: 'rejected',
};

/**
 * Where a deep link to a type (`/kortlaegning?type=<id>`) lands: the tab the
 * type lives in (Afvist for a rejected one), with it selected. An unknown id
 * has nowhere to go — null (plain screen).
 */
export function initialViewFor(types: SurveyType[], typeId: number): { tab: Tab; selection: Selection } | null {
  const t = types.find((x) => x.id === typeId);
  if (!t) return null;
  return { tab: TAB_OF[t.review_status], selection: { typeId, partCode: null } };
}

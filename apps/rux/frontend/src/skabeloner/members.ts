// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A template's member list as data (spec §5.1–5.2, §6.2): identity, the
 * editor's edits (category toggle, add a key, remove, move), the catalogue
 * search and a local port of the server's resolver. Every function returns a
 * new array, or the same array when nothing changes, and never mutates.
 *
 * `resolveMembers` must stay equal to `resolve_template` in core: the editor
 * shows its count and its struck-through members while writes are queued.
 */

import type { ResourceKey, TemplateMember } from '../api/types';
import { appendKeyMember } from '../kortlaegning/templatePick';

export function isCategory(m: TemplateMember): m is { category: string } {
  return 'category' in m;
}

export function memberId(m: TemplateMember): string {
  return isCategory(m) ? `category:${m.category}` : `key:${m.key}`;
}

export function categoryCounts(keys: readonly ResourceKey[]): { category: string; count: number }[] {
  const order: string[] = [];
  const counts = new Map<string, number>();
  for (const k of keys) {
    if (!counts.has(k.category)) order.push(k.category);
    counts.set(k.category, (counts.get(k.category) ?? 0) + 1);
  }
  return order.map((category) => ({ category, count: counts.get(category)! }));
}

export function hasCategory(members: readonly TemplateMember[], name: string): boolean {
  return members.some((m) => isCategory(m) && m.category === name);
}

export function toggleCategory(members: readonly TemplateMember[], name: string): TemplateMember[] {
  return hasCategory(members, name)
    ? members.filter((m) => !(isCategory(m) && m.category === name))
    : [...members, { category: name }];
}

/** `members` plus a key member for `keyId`, unless one is already there (R4-N3: shares Kortlægning's appendKeyMember). */
export function addKey(members: TemplateMember[], keyId: string): TemplateMember[] {
  if (members.some((m) => !isCategory(m) && m.key === keyId)) return members;
  return appendKeyMember(members, keyId);
}

export function removeMemberAt(members: TemplateMember[], index: number): TemplateMember[] {
  if (index < 0 || index >= members.length) return members;
  return members.filter((_, i) => i !== index);
}

export function moveMember(members: TemplateMember[], from: number, to: number): TemplateMember[] {
  const n = members.length;
  if (from === to || from < 0 || from >= n || to < 0 || to >= n) return members;
  const next = [...members];
  const [moved] = next.splice(from, 1);
  next.splice(to, 0, moved);
  return next;
}

export function resolveMembers(
  members: readonly TemplateMember[],
  keys: readonly ResourceKey[],
): { keys: string[]; missing: TemplateMember[] } {
  const known = new Set(keys.map((k) => k.id));
  const seen = new Set<string>();
  const out: string[] = [];
  const missing: TemplateMember[] = [];
  const push = (id: string) => {
    if (!seen.has(id)) {
      seen.add(id);
      out.push(id);
    }
  };
  for (const m of members) {
    if (isCategory(m)) {
      const inCat = keys.filter((k) => k.category === m.category);
      if (inCat.length === 0) missing.push(m);
      for (const k of inCat) push(k.id);
    } else if (known.has(m.key)) {
      push(m.key);
    } else {
      missing.push(m);
    }
  }
  return { keys: out, missing };
}

/** One catalogue-search hit: the key, plus whether a ticked category already resolves it. */
export interface SearchHit {
  key: ResourceKey;
  /** A ticked category member already includes this key — adding it as an explicit key member would pin its position but change neither the count nor the columns. */
  covered: boolean;
}

export function searchKeys(
  keys: readonly ResourceKey[],
  query: string,
  members: readonly TemplateMember[],
  limit = 20,
): SearchHit[] {
  const q = query.trim().toLocaleLowerCase('da-DK');
  if (!q) return [];
  const explicit = new Set(members.filter((m) => !isCategory(m)).map((m) => (m as { key: string }).key));
  const tickedCategories = new Set(members.filter(isCategory).map((m) => m.category));
  const hits: SearchHit[] = [];
  for (const k of keys) {
    if (explicit.has(k.id)) continue;
    const hay = `${k.label}\n${k.id}\n${k.category}`.toLocaleLowerCase('da-DK');
    if (hay.includes(q)) hits.push({ key: k, covered: tickedCategories.has(k.category) });
    if (hits.length >= limit) break;
  }
  return hits;
}

const GONE = 'Findes ikke længere';
const DELETED_COLUMN = 'Slettet felt';

export function memberLabel(
  m: TemplateMember,
  keys: readonly ResourceKey[],
): { label: string; kind: 'Kategori' | 'Felt'; detail: string } {
  if (isCategory(m)) {
    const n = keys.filter((k) => k.category === m.category).length;
    return { label: m.category, kind: 'Kategori', detail: n === 0 ? GONE : n === 1 ? '1 felt' : `${n} felter` };
  }
  const k = keys.find((x) => x.id === m.key);
  if (k) return { label: k.label, kind: 'Felt', detail: k.category };
  // A `col:` id is a deleted user column — the raw uuid means nothing to a
  // user. Anything else (`lex:`/legacy keys) still shows the raw id, since
  // that at least names the leksikon field it once pointed at.
  const label = m.key.startsWith('col:') ? DELETED_COLUMN : m.key;
  return { label, kind: 'Felt', detail: GONE };
}

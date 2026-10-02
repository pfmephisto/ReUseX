// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Resources in the Kortlægning table, as data: which columns a template
 * builds, what each cell shows and where its write goes, and how patch
 * responses fold back in. Pure, so it is testable without a DOM.
 *
 * Rulings (plan R1–R4): the page holds every resource with all its values
 * and reads a template's cells as `values[key] ?? null`; a type row shows its
 * type-scoped keys through its first part by code (a type-scoped write
 * changes every part of the type, spec §4.4); `sys:name` is the fixed tree
 * column, never a dynamic one. R3-A7: "Alle egenskaber" treats a
 * whitespace-only value as no value, same as null.
 */

import type { Resource, ResourceKey, ResourcePatchResult, SurveyPart, SurveyType, Template } from '../api/types';
import { NO_FILTERS, visibleTypes, type Filters, type Row, type Tab } from './model';

/** One template-built column of the parts table. */
export interface ResourceColumn {
  key: ResourceKey;
  /** A write changes every part of the type — the header shows a "type" marker. */
  typeScoped: boolean;
}

/** Keys drawn by a fixed column instead of a dynamic one (R4). */
export const FIXED_COLUMN_KEYS: ReadonlySet<string> = new Set(['sys:name']);

/**
 * The template's columns, in `resolved_keys` order. A key the catalogue does
 * not know (resolved after the catalogue was read) is skipped until the next
 * load; duplicates keep their first position.
 */
export function templateColumns(template: Template | null, catalogue: readonly ResourceKey[]): ResourceColumn[] {
  if (!template) return [];
  const byId = new Map(catalogue.map((k) => [k.id, k]));
  const seen = new Set<string>();
  const out: ResourceColumn[] = [];
  for (const id of template.resolved_keys) {
    if (seen.has(id) || FIXED_COLUMN_KEYS.has(id)) continue;
    seen.add(id);
    const key = byId.get(id);
    if (key) out.push({ key, typeScoped: key.scope === 'type' });
  }
  return out;
}

export type ResourceIndex = ReadonlyMap<string, Resource>;

export function resourceIndex(list: readonly Resource[]): ResourceIndex {
  return new Map(list.map((r) => [r.code, r]));
}

/** A resource's value for a key; a missing resource or key is null (blank). */
export function valueOf(index: ResourceIndex, code: string, keyId: string): string | null {
  return index.get(code)?.values[keyId] ?? null;
}

/** The part a type row reads and writes type-scoped keys through: its first by code (R3). */
export function typeCarrier(type: SurveyType): string | null {
  if (type.parts.length === 0) return null;
  return [...type.parts].map((p) => p.code).sort((a, b) => a.localeCompare(b))[0];
}

export interface CellModel {
  /** `aggregate`: the type row's Mængde sum; `none`: nothing to show. */
  kind: 'value' | 'aggregate' | 'none';
  /** The value shown and edited; null = blank. */
  value: string | null;
  /** The resource code a write goes to, or null when the cell is read-only. */
  target: string | null;
}

export function cellModel(row: Row, type: SurveyType, column: ResourceColumn, index: ResourceIndex): CellModel {
  const { key } = column;
  if (row.kind === 'part') {
    return { kind: 'value', value: valueOf(index, row.partCode, key.id), target: key.editable ? row.partCode : null };
  }
  if (key.id === 'sys:quantity') return { kind: 'aggregate', value: null, target: null };
  if (!column.typeScoped) return { kind: 'none', value: null, target: null };
  const carrier = typeCarrier(type);
  if (carrier === null) return { kind: 'value', value: null, target: null };
  return { kind: 'value', value: valueOf(index, carrier, key.id), target: key.editable ? carrier : null };
}

/** Every resource a PATCH response carries: the patched one, then its type siblings. */
export function patchedResources(result: ResourcePatchResult): Resource[] {
  return [result.resource, ...result.siblings];
}

/** `list` with each of `updated` replacing the resource of the same code (or appended). */
export function replaceResources(list: readonly Resource[], updated: readonly Resource[]): Resource[] {
  const fresh = new Map(updated.map((r) => [r.code, r]));
  const out = list.map((r) => fresh.get(r.code) ?? r);
  const known = new Set(list.map((r) => r.code));
  for (const r of updated) if (!known.has(r.code)) out.push(r);
  return out;
}

/** A write to a built-in key changed survey state: the page re-reads `/survey` (R5). */
export function touchesSurvey(keyIds: readonly string[]): boolean {
  return keyIds.some((id) => id.startsWith('sys:'));
}

/** A part added by hand (spec §4.1): no scan instance behind it. Only these can be deleted. */
export function isManual(part: SurveyPart): boolean {
  return part.instance_guid === null;
}

/** The key id of a user column definition (spec §4.3). */
export function resourceColumnKeyId(columnId: string): string {
  return `col:${columnId}`;
}

export interface PropertyGroup {
  category: string;
  keys: ResourceKey[];
}

/**
 * "Alle egenskaber": every key the resource has a value for, grouped by
 * category. Categories and keys keep catalogue order. A whitespace-only
 * value counts as no value (R3-A7).
 */
export function allPropertyGroups(resource: Resource | null, catalogue: readonly ResourceKey[]): PropertyGroup[] {
  if (!resource) return [];
  const groups = new Map<string, ResourceKey[]>();
  for (const key of catalogue) {
    const v = resource.values[key.id];
    if (v === null || v === undefined || v.trim() === '') continue;
    const list = groups.get(key.category) ?? [];
    list.push(key);
    groups.set(key.category, list);
  }
  return [...groups].map(([category, keys]) => ({ category, keys }));
}

/** The types "Tilføj ressource" offers: every type not rejected, by Danish name order. */
export function addableTypes(types: readonly SurveyType[]): SurveyType[] {
  return types.filter((t) => t.review_status !== 'rejected').sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

/** The tab and filters that show a just-created resource's type (R12). */
export function viewForNewResource(
  types: SurveyType[],
  typeId: number,
  tab: Tab,
  filters: Filters,
): { tab: Tab; filters: Filters } {
  const shown = visibleTypes(types, tab, filters).some((t) => t.id === typeId);
  return shown ? { tab, filters } : { tab: 'all', filters: NO_FILTERS };
}

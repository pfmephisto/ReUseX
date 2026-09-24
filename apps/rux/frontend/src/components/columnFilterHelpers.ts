// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PropertyDefinition } from '../api/types';

export interface NumberFilter {
  min?: number;
  max?: number;
}

export interface DateFilter {
  from?: string;
  to?: string;
}

/** null means "any" (no filter). */
export type BooleanFilter = 'true' | 'false' | null;

/** Non-empty means "value must be one of these". */
export type SelectFilter = string[];

export type ColumnFilterValue =
  | string
  | NumberFilter
  | DateFilter
  | BooleanFilter
  | SelectFilter;

export type ColumnFilters = Record<string, ColumnFilterValue>;

export function isFilterActive(val: ColumnFilterValue | undefined): boolean {
  if (val === null || val === undefined) return false;
  if (typeof val === 'string') return val.length > 0;
  if (Array.isArray(val)) return val.length > 0;
  if (typeof val === 'object') {
    const obj = val as NumberFilter & DateFilter;
    return (
      obj.min !== undefined ||
      obj.max !== undefined ||
      obj.from !== undefined ||
      obj.to !== undefined
    );
  }
  return false;
}

/** Returns true when `rawValue` passes the `filter` for `colDef`'s type. */
export function matchesColumnFilter(
  colDef: PropertyDefinition,
  rawValue: string | undefined,
  filter: ColumnFilterValue | undefined,
): boolean {
  if (!isFilterActive(filter)) return true;
  const value = rawValue ?? '';
  const f = filter!;

  if (colDef.type === 'text' || colDef.type === 'url') {
    return value.toLowerCase().includes((f as string).toLowerCase());
  }

  if (colDef.type === 'number') {
    const num = parseFloat(value);
    const nf = f as NumberFilter;
    if (isNaN(num)) {
      // An empty cell passes only when no bound is set.
      return nf.min === undefined && nf.max === undefined;
    }
    if (nf.min !== undefined && num < nf.min) return false;
    if (nf.max !== undefined && num > nf.max) return false;
    return true;
  }

  if (colDef.type === 'date') {
    const df = f as DateFilter;
    if (df.from && value < df.from) return false;
    if (df.to && value > df.to) return false;
    return true;
  }

  if (colDef.type === 'boolean') {
    const bf = f as BooleanFilter;
    if (bf === null) return true;
    return value === bf;
  }

  if (colDef.type === 'select') {
    const sf = f as SelectFilter;
    if (sf.length === 0) return true;
    return sf.includes(value);
  }

  if (colDef.type === 'multiselect') {
    const sf = f as SelectFilter;
    if (sf.length === 0) return true;
    let vals: string[] = [];
    try {
      const parsed = JSON.parse(value) as unknown;
      if (Array.isArray(parsed)) vals = parsed as string[];
    } catch {
      if (value) vals = [value];
    }
    return sf.some((sel) => vals.includes(sel));
  }

  return true;
}

export function countActiveFilters(filters: ColumnFilters): number {
  return Object.values(filters).filter(isFilterActive).length;
}

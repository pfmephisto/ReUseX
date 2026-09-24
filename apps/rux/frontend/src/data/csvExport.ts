// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pure logic for the CSV export column-selection UI (#460).
 *
 * Column names mirror `apps/ruxd/src/handlers/exports.cpp` (k_component_cols,
 * k_passport_meta_cols, and the always-present "kind" + "id"). Unknown column
 * names passed to GET /exports/csv are silently dropped by the server, so
 * user-defined property column names are also included in the selection UI —
 * they appear in the CSV when they match a passport section/property name.
 */

export interface CsvColumn {
  key: string;
  label: string;
  group: 'common' | 'component' | 'passport' | 'property';
}

/** Fixed structural columns, in declaration order from the server source. */
export const STRUCTURAL_COLUMNS: CsvColumn[] = [
  // Common identifiers (always first two in every CSV row)
  { key: 'kind', label: 'Kind', group: 'common' },
  { key: 'id', label: 'ID', group: 'common' },
  // Component-specific fields (k_component_cols)
  { key: 'component_name', label: 'Component name', group: 'component' },
  { key: 'component_type', label: 'Component type', group: 'component' },
  { key: 'confidence', label: 'Confidence', group: 'component' },
  { key: 'parent_id', label: 'Parent ID', group: 'component' },
  { key: 'notes', label: 'Notes', group: 'component' },
  { key: 'source_instance', label: 'Source instance', group: 'component' },
  { key: 'window_style', label: 'Window style', group: 'component' },
  { key: 'window_pane_count', label: 'Window pane count', group: 'component' },
  { key: 'window_operable', label: 'Window operable', group: 'component' },
  { key: 'door_style', label: 'Door style', group: 'component' },
  { key: 'door_swing', label: 'Door swing', group: 'component' },
  // Passport metadata fields (k_passport_meta_cols)
  { key: 'linked_instance', label: 'Linked instance', group: 'passport' },
  {
    key: 'passport_version_number',
    label: 'Passport version number',
    group: 'passport',
  },
  {
    key: 'passport_creation_date',
    label: 'Passport creation date',
    group: 'passport',
  },
  {
    key: 'passport_revision_date',
    label: 'Passport revision date',
    group: 'passport',
  },
  {
    key: 'passport_version_date',
    label: 'Passport version date',
    group: 'passport',
  },
];

/**
 * Convert a column selection to an export template `config.columns` array.
 *
 * Output order follows `allKeys` so the CSV column order is predictable across
 * saves. An empty selection (nothing checked) returns an empty array, which
 * the server interprets as "all columns".
 */
export function selectionToConfig(
  selected: ReadonlySet<string>,
  allKeys: ReadonlyArray<string>,
): { columns: string[] } {
  const columns = allKeys.filter((k) => selected.has(k));
  return { columns };
}

/**
 * Reconstruct a selection Set from a template config.
 *
 * When `config` is absent or its `columns` list is empty, all `allKeys` are
 * selected (= no restriction, the server returns all columns).
 * Unknown column names in `config.columns` that are not in `allKeys` are
 * silently discarded so that stale templates don't put phantom checkmarks in
 * the UI.
 */
export function configToSelection(
  config: { columns?: string[] } | undefined | null,
  allKeys: ReadonlyArray<string>,
): Set<string> {
  if (!config?.columns || config.columns.length === 0) {
    return new Set(allKeys);
  }
  const valid = new Set(allKeys);
  return new Set(config.columns.filter((k) => valid.has(k)));
}

/**
 * Build the `?columns=...` query string value from a selection.
 *
 * Returns `undefined` when the selection equals all keys — the server default
 * is already "all columns", so we omit the parameter to keep the URL clean.
 */
export function selectionToQueryParam(
  selected: ReadonlySet<string>,
  allKeys: ReadonlyArray<string>,
): string | undefined {
  const ordered = allKeys.filter((k) => selected.has(k));
  if (ordered.length === 0 || ordered.length === allKeys.length) return undefined;
  return ordered.join(',');
}

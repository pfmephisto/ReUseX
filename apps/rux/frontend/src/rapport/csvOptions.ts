// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A template's `csv` JSON as typed CSV options (spec §6.3). The backend CSV
 * builder (`GET /resources/export.csv`) reads the same object, so
 * `CSV_FIELDS` and `CSV_DEFAULTS` must equal its field names and defaults
 * (R11 / preflight P7: `resource_templates.hpp`, openapi `CsvOptions`).
 * Writing merges into the stored object and keeps keys this module does not
 * own.
 */

import { ApiRequestError } from '../api/client';
import type { Template } from '../api/types';
import { errorMessage } from '../app/saveError';

export type CsvDelimiter = ',' | ';' | '\t';
export type CsvEncoding = 'utf-8' | 'utf-8-bom';
export type CsvHeader = 'label' | 'key';

export interface CsvOptions {
  delimiter: CsvDelimiter;
  encoding: CsvEncoding;
  header: CsvHeader;
}

/** The JSON field names in `templates.csv` (Phase 1's backend). */
export const CSV_FIELDS = { delimiter: 'delimiter', encoding: 'encoding', header: 'header' } as const;

/** Matches the server's `CsvOptions` defaults (preflight P7), not the plan's original guess. */
export const CSV_DEFAULTS: CsvOptions = { delimiter: ';', encoding: 'utf-8-bom', header: 'label' };

export const DELIMITER_OPTIONS: readonly { value: CsvDelimiter; label: string }[] = [
  { value: ';', label: 'Semikolon (;)' },
  { value: ',', label: 'Komma (,)' },
  { value: '\t', label: 'Tabulator' },
];

export const ENCODING_OPTIONS: readonly { value: CsvEncoding; label: string }[] = [
  { value: 'utf-8', label: 'UTF-8' },
  { value: 'utf-8-bom', label: 'UTF-8 med BOM (Excel)' },
];

export const HEADER_OPTIONS: readonly { value: CsvHeader; label: string }[] = [
  { value: 'label', label: 'Feltnavne (fx Betegnelse)' },
  { value: 'key', label: 'Nøgle-id (fx sys:name)' },
];

function asObject(csv: unknown): Record<string, unknown> {
  return csv !== null && typeof csv === 'object' && !Array.isArray(csv) ? (csv as Record<string, unknown>) : {};
}

function pick<T extends string>(value: unknown, allowed: readonly { value: T }[], fallback: T): T {
  return allowed.some((o) => o.value === value) ? (value as T) : fallback;
}

export function readCsvOptions(csv: unknown): CsvOptions {
  const o = asObject(csv);
  return {
    delimiter: pick(o[CSV_FIELDS.delimiter], DELIMITER_OPTIONS, CSV_DEFAULTS.delimiter),
    encoding: pick(o[CSV_FIELDS.encoding], ENCODING_OPTIONS, CSV_DEFAULTS.encoding),
    header: pick(o[CSV_FIELDS.header], HEADER_OPTIONS, CSV_DEFAULTS.header),
  };
}

export function writeCsvOptions(csv: unknown, patch: Partial<CsvOptions>): Record<string, unknown> {
  const next: Record<string, unknown> = { ...asObject(csv) };
  for (const [k, v] of Object.entries(patch) as [keyof CsvOptions, string][]) next[CSV_FIELDS[k]] = v;
  return next;
}

export function downloadState(
  t: Pick<Template, 'resolved_keys'> | null,
  writing: boolean,
): { enabled: boolean; reason: string | null } {
  if (!t) return { enabled: false, reason: 'Vælg en skabelon.' };
  if (t.resolved_keys.length === 0)
    return { enabled: false, reason: 'Skabelonen har ingen felter — tilføj nogle under Skabeloner.' };
  if (writing) return { enabled: false, reason: 'Gemmer CSV-indstillingerne…' };
  return { enabled: true, reason: null };
}

/**
 * Data-eksport's count line. The CSV always leads with the part code
 * (`Kode` / `code`) ahead of the template's fields, so say so — the file
 * has one column more than the template has fields.
 */
export function csvCountLine(fields: number): string {
  return `${fields === 1 ? '1 felt' : `${fields} felter`} + kode · én række pr. ressource`;
}

/** The fallback name when the response carries no usable `Content-Disposition`. */
export const CSV_FALLBACK_FILENAME = 'ressourcer.csv';

/**
 * The download's filename from a `Content-Disposition` header value (or
 * `null`), falling back to `CSV_FALLBACK_FILENAME`. Prefers the RFC 5987
 * `filename*=UTF-8''…` form over plain `filename="…"` when both are present.
 */
export function csvFilename(contentDisposition: string | null): string {
  if (!contentDisposition) return CSV_FALLBACK_FILENAME;
  const star = /filename\*\s*=\s*[^']*''([^;]+)/i.exec(contentDisposition);
  if (star) {
    try {
      const name = decodeURIComponent(star[1].trim());
      if (name) return name;
    } catch {
      /* malformed percent-encoding — fall through to filename= */
    }
  }
  const plain = /filename\s*=\s*"?([^";]+)"?/i.exec(contentDisposition);
  const name = plain?.[1]?.trim();
  return name ? name : CSV_FALLBACK_FILENAME;
}

/** The Danish toast for a failed CSV download (a non-OK response, or the fetch itself failing). */
export function csvDownloadErrorMessage(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.isNotFound) return 'Skabelonen findes ikke længere.';
    if (cause.isRetryable) return 'Kunne ikke hente CSV-filen — serveren er ikke klar. Prøv igen om lidt.';
  }
  return `Kunne ikke hente CSV-filen: ${errorMessage(cause)}`;
}

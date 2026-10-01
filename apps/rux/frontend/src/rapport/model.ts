// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Rapport as data: how a stored version reads (date, size, title, complete or
 * draft), the hero line, the draft warning and the toasts. `blocking_types`
 * and `version` are the server's (schema v23); this only words them.
 */

import { ApiRequestError } from '../api/client';
import { parseServerUtc } from '../api/types';
import type { ReportPdfVersion, SurveyFractions, SurveySummary } from '../api/types';
import { errorMessage } from '../app/saveError';
import type { Tone } from '../kortlaegning/vocab';

/** ISO 8601 with an explicit zone: "2026-08-09T10:05:00Z", "…T12:05:00+02:00". */
const ZONED_ISO = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(?:\.\d+)?(Z|([+-])(\d{2}):(\d{2}))$/;

/**
 * sqlite's `datetime('now')` ("2026-08-09 10:05:00", UTC, no zone; seconds
 * optional) via `parseServerUtc`, or an ISO time that names its zone. Never
 * `new Date(s)`: a zone-less string would be read as local time.
 */
export function parseServerTime(s: string): Date | null {
  const t = s.trim();
  const utc = parseServerUtc(/^\d{4}-\d{2}-\d{2} \d{2}:\d{2}$/.test(t) ? `${t}:00` : t);
  if (utc) return utc;
  const m = ZONED_ISO.exec(t);
  if (!m) return null;
  const [y, mo, d, h, mi, se] = m.slice(1, 7).map(Number);
  const offsetMin = m[7] === 'Z' ? 0 : (m[8] === '-' ? -1 : 1) * (Number(m[9]) * 60 + Number(m[10]));
  const ms = Date.UTC(y, mo - 1, d, h, mi, se) - offsetMin * 60_000;
  return Number.isNaN(ms) ? null : new Date(ms);
}

/** "09.08.2026" in local time; the raw string when it does not parse. */
export function versionDate(s: string): string {
  const d = parseServerTime(s);
  return d ? d.toLocaleDateString('da-DK', { day: '2-digit', month: '2-digit', year: 'numeric' }) : s;
}

/** "09.08.2026 kl. 12.05" in local time (or `timeZone`); the raw string when it does not parse. */
export function versionDateTime(s: string, timeZone?: string): string {
  const d = parseServerTime(s);
  if (!d) return s;
  const date = d.toLocaleDateString('da-DK', { day: '2-digit', month: '2-digit', year: 'numeric', timeZone });
  const time = d.toLocaleTimeString('da-DK', { hour: '2-digit', minute: '2-digit', timeZone });
  return `${date} kl. ${time}`;
}

export function formatBytesDa(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  const kb = bytes / 1024;
  if (kb < 1024) return `${Math.round(kb).toLocaleString('da-DK')} KB`;
  return `${(kb / 1024).toLocaleString('da-DK', { minimumFractionDigits: 1, maximumFractionDigits: 1 })} MB`;
}

export function versionTitle(v: ReportPdfVersion): string {
  return `${v.label.trim() || 'Ressourcekortlægning'} — v${v.version}`;
}

export interface VersionStatus {
  tone: Tone;
  text: string;
  title: string;
}

/** Complete / draft at generation time (R7); null for a pre-v23 version. */
export function versionStatus(v: ReportPdfVersion): VersionStatus | null {
  if (v.blocking_types === null) return null;
  if (v.blocking_types === 0) {
    return { tone: 'good', text: 'Komplet', title: 'Alle typer var godkendt og afklaret, da versionen blev genereret.' };
  }
  const n = v.blocking_types;
  return {
    tone: 'wait',
    text: 'Udkast',
    title: `${n === 1 ? '1 type' : `${n} typer`} var ikke godkendt eller afventede prøvesvar.`,
  };
}

/**
 * What a version with `blocking_types: null` shows instead of a pill: it was
 * generated before the status was recorded, so it is neither complete nor a
 * draft as far as anyone can tell.
 */
export const UNKNOWN_STATUS = {
  text: 'Ukendt status',
  title: 'Versionen er ældre end statusregistreringen — det vides ikke, om den var komplet.',
} as const;

/** The hero line: "11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer". */
export function reportHeroSub(s: SurveySummary): string {
  const reuse = s.reuse_share === null ? '—' : `${Math.round(s.reuse_share * 100)} %`;
  return [
    `${s.counts.all} komponenter`,
    `${reuse} bevaring/genbrug`,
    `${s.contaminated_types} forurenet`,
    s.pending_samples === 1 ? '1 prøve afventer' : `${s.pending_samples} prøver afventer`,
  ].join(' · ');
}

export function draftNotice(f: SurveyFractions): string | null {
  if (f.ready) return null;
  const n = f.blocking_types;
  return `${n === 1 ? '1 type er' : `${n} typer er`} ikke godkendt eller afventer prøvesvar — en ny version bliver et udkast, og de indgår ikke i mængderne.`;
}

export function generatedToast(v: ReportPdfVersion): string {
  return `${versionTitle(v)} genereret${v.blocking_types ? ' (udkast)' : ''}`;
}

export function generateErrorMessage(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 409) return 'Kunne ikke generere — et pipeline-job kører. Prøv igen om lidt.';
    if (cause.status === 503) return 'Kunne ikke generere — projektet skrives til lige nu. Prøv igen om lidt.';
  }
  return `Kunne ikke generere rapporten: ${errorMessage(cause)}`;
}

/** The generation succeeded; only the re-read of the list after it failed (F22). */
export const LIST_REFRESH_FAILED = 'Rapporten blev genereret, men listen kunne ikke opdateres.';

/** The prototype's footnote without the MRK signature, which nothing models (R7). */
export const REPORT_FOOTNOTE =
  'Kun godkendte mængder indgår i rapportens kortlægningsafsnit. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel. Inventarlisten er en aktuel eksport og gemmes ikke som version.';

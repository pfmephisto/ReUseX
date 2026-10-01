// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Indberetning as data: the fraction table's rows (ready fractions, then the
 * types that block sending, both from `GET /survey/fractions`), the footer
 * status, the send gate, the CSV for manual entry in bygningsaffald.dk, and
 * the copy. The rules (bevaring left out, pending types withheld, rounding of
 * the tonnes) are the server's (R3); nothing here re-sums or re-rounds.
 */

import type { SurveyFractions } from '../api/types';
import { surveyTypeHref } from '../app/links';
import { BLOCKING_STATUS, formatTonnes, TREATMENT_LABEL, type Tone } from '../kortlaegning/vocab';

export interface ReadyRow {
  kind: 'ready';
  key: string;
  eak: string;
  fraction: string;
  contaminated: boolean;
  treatment: string;
  amount: string;
}

export interface BlockingRow {
  kind: 'blocking';
  key: string;
  typeId: number;
  /** The type in Kortlægning, selected and open, where the block is fixed. */
  href: string;
  eak: string;
  name: string;
  treatment: string;
  amount: string;
  status: { tone: Tone; text: string };
}

export type FractionRow = ReadyRow | BlockingRow;

export function fractionRows(f: SurveyFractions): FractionRow[] {
  const ready: FractionRow[] = f.fractions.map((x) => ({
    kind: 'ready',
    key: `f:${x.eak_code}:${x.treatment}:${x.contaminated ? 1 : 0}`,
    eak: x.eak_code,
    fraction: x.name || 'Ukendt EAK-kode',
    contaminated: x.contaminated,
    treatment: TREATMENT_LABEL[x.treatment],
    amount: formatTonnes(x.mass_t),
  }));
  const blocking: FractionRow[] = f.blocking.map((b) => ({
    kind: 'blocking',
    key: `b:${b.type_id}`,
    typeId: b.type_id,
    href: surveyTypeHref(b.type_id),
    eak: b.eak_code || '—',
    name: b.name,
    treatment: TREATMENT_LABEL[b.treatment],
    amount: b.mass_t === null ? '(—)' : `(${formatTonnes(b.mass_t)})`,
    status: BLOCKING_STATUS[b.reason],
  }));
  return [...ready, ...blocking];
}

/**
 * The send gate (R9): open exactly when there is a fraction to report and no
 * type blocks. The lists are the evidence, so a `ready` flag that disagrees
 * with them never opens the gate; an empty survey has nothing to send.
 */
export function canSend(f: SurveyFractions): boolean {
  return f.fractions.length > 0 && f.blocking.length === 0;
}

/** The status when nothing blocks but there is no fraction to report either. */
export const NOTHING_TO_SEND = 'Ingen fraktioner at indberette';

/** The footer status pill's id; the disabled Send button is described by it. */
export const FOOT_STATUS_ID = 'indberetning-foot-status';

export function footStatus(f: SurveyFractions): { tone: Tone; text: string } {
  if (canSend(f)) return { tone: 'good', text: 'Klar til afsendelse' };
  if (f.blocking.length === 0) return { tone: 'wait', text: NOTHING_TO_SEND };
  const n = f.blocking_types;
  return { tone: 'warn', text: n === 1 ? '1 type blokerer' : `${n} typer blokerer` };
}

function csvField(s: string): string {
  return /[";\r\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

/**
 * A text cell Excel would evaluate (`=`, `+`, `-`, `@`, tab, CR first) gets a
 * leading `'`, so a type named `=HYPERLINK(…)` stays text (CSV injection).
 * Only for text columns: the tonnes are formatted here and never start so.
 */
function csvText(s: string): string {
  return csvField(/^[=+\-@\t\r]/.test(s) ? `'${s}` : s);
}

/** The wire rounds tonnes to 6 dp; the CSV keeps every one of those digits. */
const CSV_TONNE_DIGITS = 6;

/**
 * The ready fractions in the portal's columns: `;`-separated with a decimal
 * comma and CRLF, which Danish Excel opens without an import dialog.
 */
export function fractionsCsv(f: SurveyFractions): string {
  const lines = ['EAK-kode;Fraktion;Behandling;Forurenet;Mængde (t)'];
  for (const x of f.fractions) {
    lines.push(
      [
        ...[x.eak_code, x.name, TREATMENT_LABEL[x.treatment], x.contaminated ? 'Ja' : 'Nej'].map(csvText),
        csvField(x.mass_t.toLocaleString('da-DK', { useGrouping: false, maximumFractionDigits: CSV_TONNE_DIGITS })),
      ].join(';'),
    );
  }
  return `${lines.join('\r\n')}\r\n`;
}

/** UTF-8 byte-order mark: without it Excel reads the file as cp1252 and garbles æøå. */
const BOM = '\uFEFF';

/** The download link's target: the CSV behind a BOM, as a data URL. */
export function fractionsCsvHref(f: SurveyFractions): string {
  return `data:text/csv;charset=utf-8,${encodeURIComponent(BOM + fractionsCsv(f))}`;
}

export const CSV_FILENAME = 'fraktioner.csv';

/** The prototype's note, with the bevaring rule said out loud (R3). */
export const FRACTION_NOTE =
  'Fraktionerne herunder er aggregeret pr. EAK-kode og behandling — kun godkendte mængder tælles med, og bevaring indgår ikke, da den bliver i bygningen. Rækker der afventer gennemsyn eller miljøsvar, eller mangler tons, er vist nederst og blokerer afsendelse. Direkte indberetning kommer senere — v1 giver tallene i portalens struktur.';

export const SEND_NOTICE =
  'Ikke sendt. Direkte indberetning til bygningsaffald.dk er ikke koblet på endnu — hent tallene som CSV og indtast dem i portalen.';

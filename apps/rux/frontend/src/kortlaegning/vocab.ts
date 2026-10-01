// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Danish words and number formats Kortlægning shows. The wire vocabulary
 * (core/survey.hpp, docs/gui/openapi.yaml) stays ASCII; this is the only place
 * it becomes user-facing copy.
 */

import type { EnvironmentStatus, SampleStage, Treatment } from '../api/types';

export type Tone = 'good' | 'warn' | 'wait' | 'crit' | 'accent';

export const TREATMENT_LABEL: Record<Treatment, string> = {
  bevaring: 'Bevaring',
  genbrug: 'Genbrug',
  genanvendelse: 'Genanvendelse',
  nyttiggoerelse: 'Nyttiggørelse',
  bortskaffelse: 'Bortskaffelse',
};

export const ENV_LABEL: Record<EnvironmentStatus, string> = {
  ren_screening: 'Ren',
  afventer: 'Afventer prøve',
  forurenet: 'Forurenet',
  ren_proevesvar: 'Ren (prøvesvar)',
};

export const ENV_TONE: Record<EnvironmentStatus, Tone> = {
  ren_screening: 'good',
  afventer: 'wait',
  forurenet: 'crit',
  ren_proevesvar: 'good',
};

export const STAGE_LABEL: Record<SampleStage, string> = {
  planlagt: 'Planlagt',
  udtaget: 'Udtaget',
  sendt: 'Sendt til lab',
  svar: 'Svar modtaget',
};

/** The waste-hierarchy colour token for a treatment. */
export function circToken(t: Treatment): string {
  return `var(--circ-${t})`;
}

export function formatNumber(n: number, maxFractionDigits = 1): string {
  return n.toLocaleString('da-DK', { maximumFractionDigits: maxFractionDigits });
}

export function formatQuantity(q: number, unit: string): string {
  return `${formatNumber(q)} ${unit}`;
}

export function formatTonnes(t: number | null): string {
  return t === null ? '' : `${formatNumber(t)} t`;
}

export function confidencePercent(c: number | null): number | null {
  return c === null ? null : Math.round(c * 100);
}

/**
 * Parse a quantity as a Danish user types it: '.' groups thousands, ',' is the
 * decimal point. Returns null for empty, malformed or negative input.
 */
export function parseDanishNumber(text: string): number | null {
  const t = text.trim().replace(/\./g, '').replace(',', '.');
  if (t === '' || !/^\d+(\.\d+)?$/.test(t)) return null;
  return Number(t);
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The links between the case screens, as data: a sample line in
 * Kortlægning opens its sample; a type with no sample opens the create form
 * pre-linked; a sample's linked type opens that type. Paths stay
 * extensionless (see App.tsx) and ids are positive integers.
 */

export const MILJOE_PATH = '/miljoe';
export const KORTLAEGNING_PATH = '/kortlaegning';
export const OVERBLIK_PATH = '/';
export const RAPPORT_PATH = '/rapport';
export const INDBERETNING_PATH = '/indberetning';
export const PROJEKTDATA_PATH = '/projektdata';

/** Skabeloner — the template editor (resources/templates spec §6.2). Its page arrives in Phase 4. */
export const SKABELONER_PATH = '/skabeloner';

export function sampleHref(sampleId: number): string {
  return `${MILJOE_PATH}?sample=${sampleId}`;
}

export function newSampleHref(typeId: number): string {
  return `${MILJOE_PATH}?ny=${typeId}`;
}

export function surveyTypeHref(typeId: number): string {
  return `${KORTLAEGNING_PATH}?type=${typeId}`;
}

function positiveId(value: string | null): number | null {
  if (value === null || !/^\d+$/.test(value)) return null;
  const n = Number(value);
  return n > 0 && Number.isSafeInteger(n) ? n : null;
}

export interface MiljoeQuery {
  /** `?sample=<id>`: scroll to and highlight that card. */
  sampleId: number | null;
  /** `?ny=<typeId>`: open the create form with that type pre-linked. */
  newForType: number | null;
}

export function parseMiljoeQuery(search: string): MiljoeQuery {
  const q = new URLSearchParams(search);
  return { sampleId: positiveId(q.get('sample')), newForType: positiveId(q.get('ny')) };
}

/** `?type=<id>` on /kortlaegning. */
export function parseTypeQuery(search: string): number | null {
  return positiveId(new URLSearchParams(search).get('type'));
}

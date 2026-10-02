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

export const ONSITE_PATH = '/on-site';

/** On-site at one bygningsdel. */
export function onsiteHref(code: string): string {
  return `${ONSITE_PATH}?del=${encodeURIComponent(code)}`;
}

/** `?del=<RX-###>` on /on-site; anything that is not a part code is ignored. */
export function parseOnsiteQuery(search: string): string | null {
  const v = new URLSearchParams(search).get('del');
  return v !== null && /^RX-\d{1,6}$/.test(v) ? v : null;
}

/** The raw `?del=` value, malformed or not, so the page can name what it was asked for; null when absent or empty. */
export function rawOnsiteDel(search: string): string | null {
  const v = new URLSearchParams(search).get('del');
  return v ? v : null;
}

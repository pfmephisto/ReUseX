// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The 360° evidence view as data (spec A5): which panorama to show for a part,
 * what the tab says when there is none, and the arithmetic of the pannable
 * equirect strip — where the image sits so the part's `u` is centred, and
 * where the marker goes as the user pans. Pure, so the "centred on the part"
 * rule is a tested claim rather than a CSS accident.
 */

import type { InstancePanorama } from '../api/types';

/** Edge length asked of the server for the pannable strip (an equirect is 2:1). */
export const PANO_STRIP_MAX_SIZE = 2048;
/** Edge length for the dialog's 360° thumbnail. */
export const PANO_THUMB_MAX_SIZE = 480;

export const PANO_TEXT = {
  unlinked: 'Ingen 360° — bygningsdelen er ikke koblet til en instans.',
  loading: 'Finder nærmeste 360°-optagelse…',
  failed: '360°-optagelserne kunne ikke hentes.',
  none: 'Ingen 360°-optagelse nær denne ressource',
} as const;

/** A panorama lookup's result, tagged with the highlight it was fetched for. */
export interface PanoLookup {
  key: string;
  panoramas: InstancePanorama[];
  failed: boolean;
}

/**
 * The panorama to show from a nearest-first list: the nearest **resected**
 * one (`rux align 360` measured its heading, so `u` really points at the
 * part), else the nearest levelled one (its heading is a guess, so it is shown
 * without a marker). Measured on the NewOffice project: the nearest panorama
 * to a part is often levelled while a resected one stands a few metres
 * further off, and a marker on a guessed heading points at the wrong wall.
 */
export function pickPano(panoramas: readonly InstancePanorama[]): InstancePanorama | null {
  return panoramas.find((p) => p.heading === 'resected') ?? panoramas[0] ?? null;
}

/**
 * The panorama to show for the current highlight, or why there is none.
 * A lookup for another key is treated as still loading (see `resolvePhotoState`
 * in `photo.ts` for why `useAsync`'s stale data makes this necessary).
 */
export function resolvePano(
  currentKey: string | null,
  data: PanoLookup | undefined,
): { pano: InstancePanorama | null; empty: string } {
  if (currentKey === null) return { pano: null, empty: PANO_TEXT.unlinked };
  if (!data || data.key !== currentKey) return { pano: null, empty: PANO_TEXT.loading };
  if (data.failed) return { pano: null, empty: PANO_TEXT.failed };
  const pano = pickPano(data.panoramas);
  return { pano, empty: pano ? '' : PANO_TEXT.none };
}

/**
 * `<rum> · 360°`, or just `360°` for a part with no room; a levelled panorama
 * adds that its heading is unknown (no marker is drawn for it).
 */
export function panoCaption(roomName: string | null | undefined, heading: InstancePanorama['heading'] = 'resected'): string {
  const room = roomName?.trim();
  const base = room ? `${room} · 360°` : '360°';
  return heading === 'levelled' ? `${base} · retning ukendt` : base;
}

/** Width of the equirect drawn at @p height (equirects are 2:1). */
export function panoImageWidth(height: number): number {
  return 2 * height;
}

/** `x` wrapped into `[-period/2, period/2)`. */
export function wrapCentred(x: number, period: number): number {
  if (!(period > 0)) return 0;
  const half = period / 2;
  return ((((x + half) % period) + period) % period) - half;
}

/**
 * `background-position-x` (px) of the repeating equirect so that column
 * `u` sits at the middle of a strip @p viewWidth wide, shifted by the user's
 * @p pan (px, positive = dragged right). The image repeats horizontally, so
 * any value is valid; it is wrapped to one period to keep the number small.
 */
export function panoBackgroundX(u: number, viewWidth: number, imageWidth: number, pan: number): number {
  if (!(imageWidth > 0)) return 0;
  const raw = viewWidth / 2 - u * imageWidth + pan;
  return ((raw % imageWidth) + imageWidth) % imageWidth;
}

/**
 * Left offset (px) of the part's marker: the strip's middle plus the pan,
 * wrapped so the marker stays on the copy of the image nearest the middle.
 */
export function panoMarkerX(viewWidth: number, imageWidth: number, pan: number): number {
  return viewWidth / 2 + wrapCentred(pan, imageWidth);
}

/** Pan step (px) for one ArrowLeft/ArrowRight press on the focused strip. */
export function panoKeyStep(viewWidth: number): number {
  return Math.max(1, Math.round(viewWidth / 8));
}

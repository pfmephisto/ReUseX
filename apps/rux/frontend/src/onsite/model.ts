// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * On-site as data: the walk through the stored bygningsdele (R6), the
 * detection chip and photo stage, and what the sheet sends (R7). Pure, so the
 * walk and every write's shape are testable without a DOM.
 *
 * Built on the existing rules rather than copies of them: room names from
 * Kortlægning's `roomName`, the photo lookup from `kortlaegning/photo`, the
 * sample body from Miljø's `createBody` and the gate wording from Miljø's
 * `gatePhrases`.
 */

import type { Sample, SampleCreate, SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import { roomName } from '../kortlaegning/model';
import { type FrameLookup, resolvePhotoState } from '../kortlaegning/photo';
import { confidencePercent } from '../kortlaegning/vocab';
import { createBody, type GateChange, gatePhrases } from '../miljoe/model';

/** One bygningsdel on the walk. `room` is '' for a part without one. */
export interface Stop {
  code: string;
  typeId: number;
  room: string;
}

/** The picker's group for parts without a room. */
export const NO_ROOM = 'Uden rum';

/**
 * Every part of every non-rejected type, room by room (Danish collation),
 * then by code (numeric). Parts without a room come last.
 */
export function walkOrder(types: readonly SurveyType[]): Stop[] {
  const stops: Stop[] = [];
  for (const t of types) {
    if (t.review_status === 'rejected') continue;
    for (const p of t.parts) stops.push({ code: p.code, typeId: t.id, room: roomName(p) });
  }
  return stops.sort((a, b) => {
    if ((a.room === '') !== (b.room === '')) return a.room === '' ? 1 : -1;
    return a.room.localeCompare(b.room, 'da') || a.code.localeCompare(b.code, 'da', { numeric: true });
  });
}

/** The asked part when it is on the walk, else the first stop; null for an empty walk. */
export function currentStop(order: readonly Stop[], asked: string | null): Stop | null {
  if (order.length === 0) return null;
  return order.find((s) => s.code === asked) ?? order[0];
}

/** How much of a raw `?del=` value the notice repeats. */
const NOTICE_MAX = 40;

/**
 * Said when `?del=` names a part that is not on the walk: unknown, of a
 * rejected type, or not a part code at all. `asked` is the raw value, so a
 * malformed one is named too; the notice says where the page went instead.
 */
export function unknownNotice(asked: string | null, stop: Stop | null): string | null {
  if (asked === null || asked === '' || stop?.code === asked) return null;
  const shown = asked.length > NOTICE_MAX ? `${asked.slice(0, NOTICE_MAX)}…` : asked;
  const base = `Bygningsdel '${shown}' findes ikke`;
  return stop ? `${base} — viser ${stop.code}.` : `${base}.`;
}

/**
 * The stop after `code`, wrapping at the end. Null when there is nothing else
 * to go to, or when `code` is not on the walk.
 */
export function stopAfter(order: readonly Stop[], code: string): Stop | null {
  if (order.length < 2) return null;
  const at = order.findIndex((s) => s.code === code);
  if (at < 0) return null;
  return order[(at + 1) % order.length];
}

export function nextLabel(next: Stop | null): string {
  return next ? `Videre → ${next.code}` : 'Ingen flere bygningsdele';
}

export function partAt(types: readonly SurveyType[], stop: Stop | null): { type: SurveyType; part: SurveyPart } | null {
  if (!stop) return null;
  const type = types.find((t) => t.id === stop.typeId);
  const part = type?.parts.find((p) => p.code === stop.code);
  return type && part ? { type, part } : null;
}

export interface PickerGroup {
  room: string;
  options: { code: string; label: string }[];
}

/** The part select's option groups: one per room, in walk order. */
export function pickerGroups(order: readonly Stop[], types: readonly SurveyType[]): PickerGroup[] {
  const names = new Map(types.map((t) => [t.id, t.name]));
  const groups: PickerGroup[] = [];
  for (const s of order) {
    const room = s.room || NO_ROOM;
    let group = groups.at(-1);
    if (!group || group.room !== room) {
      group = { room, options: [] };
      groups.push(group);
    }
    const name = names.get(s.typeId);
    group.options.push({
      code: s.code,
      label: name ? `${s.code} · ${name}` : s.code,
    });
  }
  return groups;
}

/** The chip's small line: code, room and the type's AI confidence, whichever exist. */
export function chipDetail(type: SurveyType, part: SurveyPart): string {
  const bits = [part.code];
  const room = roomName(part);
  if (room) bits.push(room);
  const c = confidencePercent(type.confidence);
  if (c !== null) bits.push(`sikkerhed ${c} %`);
  return bits.join(' · ');
}

/** A frames lookup tagged with the instance it was made for — Kortlægning's `FrameLookup`. */
export type PhotoLookup = FrameLookup;

export type PhotoView =
  | { kind: 'unlinked' }
  | { kind: 'loading' }
  | { kind: 'failed' }
  | { kind: 'none' }
  | { kind: 'photo'; frame: VisibleFrame };

/**
 * What the stage shows. `currentKey` is the current part's instance key, or
 * null without an instance link. A lookup made for another key — the previous
 * part's, in the render right after `Videre` — counts as still loading, never
 * as this part's photo (`resolvePhotoState`'s rule).
 */
export function photoView(currentKey: string | null, lookup: PhotoLookup | undefined): PhotoView {
  if (currentKey === null) return { kind: 'unlinked' };
  const { photoFrameId, photoFailed } = resolvePhotoState(currentKey, lookup);
  if (photoFailed) return { kind: 'failed' };
  if (photoFrameId === undefined) return { kind: 'loading' };
  const frame = lookup?.frames[0];
  return photoFrameId !== null && frame ? { kind: 'photo', frame } : { kind: 'none' };
}

export const PHOTO_TEXT: Record<Exclude<PhotoView['kind'], 'photo'>, string> = {
  unlinked: 'Intet foto — bygningsdelen er ikke koblet til en instans.',
  loading: 'Indlæser foto…',
  failed: 'Foto kunne ikke hentes.',
  none: 'Intet foto — der blev ikke fundet en ramme for denne instans.',
};

/** Percent of the stage. */
export interface Reticle {
  left: number;
  top: number;
  width: number;
  height: number;
}

/** The prototype's reticle size, as a share of the stage. */
const RETICLE_W = 0.48;
const RETICLE_H = 0.4;

function pct(share: number): number {
  return Math.round(share * 1000) / 10;
}

/**
 * The reticle around the instance centroid's projection (`u`, `v`, in the
 * frame's pixels), clamped inside the stage. Null without a frame, without
 * the frame size, or when the projection is not a point inside the frame
 * (out of range or NaN).
 */
export function reticleBox(
  frame: Pick<VisibleFrame, 'u' | 'v'> | undefined,
  size: { width?: number; height?: number },
): Reticle | null {
  if (!frame || !size.width || !size.height) return null;
  const cx = frame.u / size.width;
  const cy = frame.v / size.height;
  if (!(cx >= 0 && cx <= 1 && cy >= 0 && cy <= 1)) return null;
  const left = Math.min(1 - RETICLE_W, Math.max(0, cx - RETICLE_W / 2));
  const top = Math.min(1 - RETICLE_H, Math.max(0, cy - RETICLE_H / 2));
  return {
    left: pct(left),
    top: pct(top),
    width: pct(RETICLE_W),
    height: pct(RETICLE_H),
  };
}

export function starButton(starred: boolean): { icon: string; text: string } {
  return starred ? { icon: '★', text: 'Vigtig — tryk for at fjerne' } : { icon: '☆', text: 'Markér som vigtig' };
}

/**
 * The `POST /samples` body for a sample registered at a part (R7, R8): Miljø's
 * body, already taken, at the part. Null while the title is empty.
 */
export function onsiteSampleBody(title: string, what: string, part: SurveyPart): SampleCreate | null {
  const body = createBody(title, what, [part.type_id]);
  return body && { ...body, part_code: part.code, stage: 'udtaget' };
}

/**
 * The toast after a sample is registered. A sample that is not yet answered
 * can only make types wait, so only `blocked` / `reblocked` are worded, in
 * Miljø's words.
 */
export function sampleToast(code: string, partCode: string, gate: GateChange): string {
  const base = `✓ ${code} registreret ved ${partCode}`;
  const effect = gatePhrases({
    unblocked: [],
    contaminated: [],
    blocked: gate.blocked,
    reblocked: gate.reblocked,
    released: [],
  });
  return effect.length > 0 ? `${base} — ${effect.join('; ')}` : base;
}

/** The toast when a sample was registered but the survey could not be re-read for its gate effect. */
export function sampleReloadFailedToast(code: string, partCode: string): string {
  return `✓ ${code} registreret ved ${partCode} — men miljøstatus kunne ikke genindlæses.`;
}

/**
 * The empty state when the walk has no stops: no parts at all, or parts only
 * on rejected types (which the walk leaves out).
 */
export function emptyWalkText(types: readonly SurveyType[]): { title: string; detail: string } {
  if (types.some((t) => t.parts.length > 0)) {
    return {
      title: 'Alle bygningsdele er afvist',
      detail:
        'Bygningsdelene hører kun til afviste typer, så der er intet at gå til. En afvisning fortrydes i Kortlægning.',
    };
  }
  return {
    title: 'Ingen bygningsdele endnu',
    detail: 'Bygningsdele oprettes i Kortlægning (Opret kortlægning), ud fra projektets instanser.',
  };
}

/** The samples on the part's type, in list order, each marked when it was taken at this part. */
export function typeSamples(
  type: SurveyType,
  part: SurveyPart,
  samples: readonly Sample[],
): { sample: Sample; here: boolean }[] {
  return samples
    .filter((s) => s.type_ids.includes(type.id))
    .map((s) => ({ sample: s, here: s.part_code === part.code }));
}

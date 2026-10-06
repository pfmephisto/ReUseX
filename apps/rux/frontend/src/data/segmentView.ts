// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Segmentering view (spec B3) as pure data: the frame filmstrip, prompts
 * in image pixels, the request they become, the per-prompt result list, the
 * mask overlay pixels, and the "Opret ressource fra markering" dialog's
 * choices. The page only wires these to React and the canvas.
 *
 * Prompts live in **image pixel** coordinates (the colour image's natural
 * size) rather than display pixels, so zoom-to-fit and window resizes never
 * move a drawn box, and a seed pixel from the URL drops in unchanged.
 */

import type {
  FrameInfo,
  FrameSegmentPrompt,
  FrameSegmentResult,
  SegmentResourceRequest,
  SurveyType,
} from '../api/types';
import { labelColorIndex } from '../viewport/labelColors';
import type { LabelImage } from './labelPng';
import { POINT_CLICK_RADIUS } from './segmentPrompts';

/** `[x1, y1, x2, y2]` in image pixels, x1 <= x2, y1 <= y2. */
export type ImageBox = [number, number, number, number];

export interface SegPrompt {
  id: string;
  /** Class name; may be empty when `box` is set (sent as SAM3's "visual"). */
  text: string;
  box: ImageBox | null;
  /** The box is a clicked point's marker (drawn as a dot; the click itself is `at`). */
  point: boolean;
  /** The clicked pixel of a point prompt, exactly (the marker box is clamped to the image). */
  at?: [number, number];
}

/** The server's text for a box-only prompt (`kGeometryOnlyPromptText`). */
export const GEOMETRY_ONLY_TEXT = 'visual';

// ------------------------------------------------------------ filmstrip --

/** `ids` around `current`, `radius` on each side, clamped to the ends (window keeps its width). */
export function filmstripWindow(ids: readonly number[], current: number | null, radius: number): number[] {
  if (ids.length === 0) return [];
  const width = 2 * radius + 1;
  if (ids.length <= width) return [...ids];
  const at = current === null ? 0 : Math.max(0, ids.indexOf(current));
  const start = Math.min(Math.max(0, at - radius), ids.length - width);
  return ids.slice(start, start + width);
}

/** The id `delta` steps from `current` in `ids`, clamped; null for an empty list. */
export function stepFrame(ids: readonly number[], current: number | null, delta: number): number | null {
  if (ids.length === 0) return null;
  const at = current === null ? -1 : ids.indexOf(current);
  if (at === -1) return ids[0];
  return ids[Math.min(ids.length - 1, Math.max(0, at + delta))];
}

// ---------------------------------------------------------- geometry --

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

/** A clicked point's marker: a small square around it, clamped to the image (drawn as a dot). */
export function pointBox(u: number, v: number, width: number, height: number, radius = POINT_CLICK_RADIUS): ImageBox {
  return [
    clamp(Math.round(u - radius), 0, width - 1),
    clamp(Math.round(v - radius), 0, height - 1),
    clamp(Math.round(u + radius), 0, width - 1),
    clamp(Math.round(v + radius), 0, height - 1),
  ];
}

/** A point marker's centre — the click, unless the marker was clamped at an edge (prefer `SegPrompt.at`). */
export function pointOf(box: ImageBox): [number, number] {
  return [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2];
}

/** Upper bound for the Før/Efter neighbour counts ("Tilføj til kø"). */
export const NEIGHBOR_MAX = 500;

/** A Før/Efter input value as a whole count in 0..NEIGHBOR_MAX (junk → 0). */
export function clampNeighborCount(raw: string): number {
  const n = Math.trunc(Number(raw));
  if (!Number.isFinite(n)) return 0;
  return Math.min(NEIGHBOR_MAX, Math.max(0, n));
}

/** Two corners in any order → a normalised, clamped, integer box. */
export function cornersToBox(
  a: { x: number; y: number },
  b: { x: number; y: number },
  width: number,
  height: number,
): ImageBox {
  return [
    clamp(Math.round(Math.min(a.x, b.x)), 0, width - 1),
    clamp(Math.round(Math.min(a.y, b.y)), 0, height - 1),
    clamp(Math.round(Math.max(a.x, b.x)), 0, width - 1),
    clamp(Math.round(Math.max(a.y, b.y)), 0, height - 1),
  ];
}

/**
 * A seed pixel from the URL (`/frames/visibility` u,v: the intrinsics' pixel
 * grid) in the colour image's pixels, or null when it falls outside.
 */
export function seedToImage(
  seed: { u: number; v: number },
  intrinsics: { width: number; height: number } | undefined,
  image: { width: number; height: number },
): { x: number; y: number } | null {
  const sx = intrinsics && intrinsics.width > 0 ? image.width / intrinsics.width : 1;
  const sy = intrinsics && intrinsics.height > 0 ? image.height / intrinsics.height : 1;
  const x = seed.u * sx;
  const y = seed.v * sy;
  if (x < 0 || y < 0 || x >= image.width || y >= image.height) return null;
  return { x, y };
}

// ------------------------------------------------------------ request --

export interface SentPrompt {
  promptId: string;
  /** Trimmed text as sent ("" for a box-only prompt). */
  text: string;
}

/**
 * The prompts as the request carries them, plus what was sent in order — the
 * order *is* the label value in the saved mask (prompt index), so the result
 * list and `mask_label` both come from `sent`, never from the live prompt
 * list the user may have edited since.
 */
export function buildRequestPrompts(prompts: readonly SegPrompt[]): {
  prompts: FrameSegmentPrompt[];
  sent: SentPrompt[];
} {
  const out: FrameSegmentPrompt[] = [];
  const sent: SentPrompt[] = [];
  for (const p of prompts) {
    const text = p.text.trim();
    if (!text && !p.box) continue; // an empty text-only prompt is nothing
    const prompt: FrameSegmentPrompt = { text };
    if (p.box && p.point) prompt.points = [p.at ?? pointOf(p.box)];
    else if (p.box) prompt.boxes = [['pos', p.box]];
    out.push(prompt);
    sent.push({ promptId: p.id, text });
  }
  return { prompts: out, sent };
}

// ------------------------------------------------------------- result --

export interface ResultClass {
  /** Prompt index = label value in the saved mask (API encoding). */
  index: number;
  promptId: string | null;
  /** What the list shows. */
  name: string;
  /** Prefill for the resource dialog's class name ("" for a box-only prompt). */
  className: string;
  /** Pixels with this label; null until the mask has been read. */
  pixels: number | null;
}

/**
 * One row per class the run reported. Prompts sent map by index; a run with
 * no prompts (the model's default classes) lists `result.labels` as is.
 */
export function resultClasses(
  result: FrameSegmentResult,
  sent: readonly SentPrompt[],
  counts: ReadonlyMap<number, number> | null,
): ResultClass[] {
  const indices = new Set<number>(sent.map((_, i) => i));
  for (const key of Object.keys(result.labels)) {
    const n = Number(key);
    if (Number.isInteger(n) && n >= 0) indices.add(n);
  }
  return [...indices]
    .sort((a, b) => a - b)
    .map((index) => {
      const s = sent[index];
      const serverName = result.labels[String(index)] ?? '';
      const text = s ? s.text : serverName === GEOMETRY_ONLY_TEXT ? '' : serverName;
      return {
        index,
        promptId: s?.promptId ?? null,
        name: text || `Markering ${index + 1}`,
        className: text,
        pixels: counts ? (counts.get(index) ?? 0) : null,
      };
    });
}

/**
 * A note when a drawn point or box found nothing. Points and boxes reach
 * SAM3 (the server keeps the object under a point, or the objects a box
 * covers), so an empty result means SAM3 saw no object there.
 */
export function geometryHint(classes: readonly ResultClass[], prompts: readonly SegPrompt[]): string | null {
  const byId = new Map(prompts.map((p) => [p.id, p]));
  const empty = classes.filter((c) => c.promptId !== null && byId.get(c.promptId)?.box && c.pixels === 0);
  const point = empty.find((c) => byId.get(c.promptId!)?.point);
  const hit = point ?? empty[0];
  if (!hit) return null;
  const name = hit.className;
  if (point) {
    return name
      ? `Punktet ramte ingen ${name}. Klik midt på den, eller træk en boks om den.`
      : 'Punktet ramte intet objekt. Klik midt på objektet, træk en boks om det, eller skriv et klassenavn.';
  }
  return name
    ? `Boksen fandt ingen ${name}. Træk den om hele objektet, eller prøv en lavere konfidens.`
    : 'Boksen fandt intet objekt. Træk den tættere om objektet, eller skriv et klassenavn.';
}

/**
 * A quiet note when the run's boxes or points did not reach SAM3 (ONNX
 * backend, text-only engines, a geometry path that failed): the detections
 * were then only clipped to the boxes, so a click can come back empty for a
 * reason the user cannot see otherwise. Null when no prompt carried geometry,
 * when it reached the model, or when the server did not say (older server).
 */
export function geometryNotice(result: FrameSegmentResult, sent: readonly FrameSegmentPrompt[]): string | null {
  const hadGeometry = sent.some((p) => (p.boxes?.length ?? 0) > 0 || (p.points?.length ?? 0) > 0);
  if (!hadGeometry || result.geometry_prompts_used !== false) return null;
  return 'Bokse og punkter nåede ikke modellen her, så fundene er kun beskåret til boksene. Skriv et klassenavn for et sikrere resultat.';
}

/**
 * The toast for a run that failed after the user moved to another frame: the
 * page no longer shows that frame's error line, so the failure is named here.
 */
export function offscreenRunError(frameId: number, message: string): string {
  return `Segmentering af billede ${frameId} fejlede: ${message}`;
}

/** Why "Opret ressource fra markering" cannot run on this frame, or null. */
export function resourceBlockReason(frame: FrameInfo | undefined): string | null {
  if (!frame) return 'Billedet indlæses…';
  if (frame.has_pose === false) return 'Billedet har ingen gemt kameraposition, så markeringen kan ikke placeres i punktskyen.';
  if (frame.has_depth === false) return 'Billedet har intet dybdebillede, så markeringen kan ikke placeres i punktskyen.';
  return null;
}

// --------------------------------------------------------------- mask --

/**
 * RGBA pixels for the mask overlay: each prompt in its `--label-N` colour,
 * unlabeled transparent. With a `selected` prompt that one is drawn strong
 * and the others faint, so "which pixels is this class" reads at a glance.
 * Alphas are 0..255.
 */
export function maskRgba(
  image: LabelImage,
  colors: readonly [number, number, number][],
  selected: number | null,
  alpha: { normal: number; strong: number; faint: number },
): Uint8ClampedArray<ArrayBuffer> {
  const out = new Uint8ClampedArray(image.width * image.height * 4);
  if (colors.length === 0) return out;
  const rgb = colors.map(([r, g, b]) => [Math.round(r * 255), Math.round(g * 255), Math.round(b * 255)]);
  for (let i = 0; i < image.data.length; i += 1) {
    const v = image.data[i];
    if (v === 0) continue;
    const c = rgb[labelColorIndex(v, rgb.length)];
    const o = i * 4;
    out[o] = c[0];
    out[o + 1] = c[1];
    out[o + 2] = c[2];
    out[o + 3] = selected === null ? alpha.normal : v - 1 === selected ? alpha.strong : alpha.faint;
  }
  return out;
}

// ------------------------------------------------------ resource dialog --

/** `'new'` = no type_id: the server picks the class's type or makes one named after it. */
export type TypeChoice = number | 'new';

/** Types offered in the dialog: everything not rejected, by name. */
export function selectableTypes(types: readonly SurveyType[]): SurveyType[] {
  return types
    .filter((t) => t.review_status !== 'rejected')
    .slice()
    .sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

/**
 * The type the server would file the part in when no `type_id` is sent, or
 * `'new'` when it would create one — mirroring `core::apply_mask_selection`:
 * an existing class (by exact name in the `labels` definitions, ids >= 1)
 * with a type of that `semantic_class`, else a type named exactly like the
 * class. Rejected (Afvist) types are skipped by both matches, as on the
 * server: when only a rejected type matches, a new type is created. The
 * dialog preselects it and offers "Ny type" only when it is `'new'`, so the
 * option never promises a type the server would not create.
 */
export function defaultTypeChoice(
  types: readonly SurveyType[],
  className: string,
  labelNames: Readonly<Record<string, string>> | null,
): TypeChoice {
  const name = className.trim();
  if (!name) return 'new';
  const live = types.filter((t) => t.review_status !== 'rejected');
  for (const [id, label] of Object.entries(labelNames ?? {})) {
    if (Number(id) < 1 || label !== name) continue;
    const bySemantic = live.find((t) => t.semantic_class === Number(id));
    if (bySemantic) return bySemantic.id;
  }
  const byName = live.find((t) => t.name === name);
  return byName ? byName.id : 'new';
}

/**
 * What the dialog's type `<select>` shows *and* submits: the user's pick
 * while it is one of the rendered options, else the automatic choice — and
 * never a value without an option (a browser would show the first option
 * while the form sent the hidden value). "Ny type" is rendered only when
 * `automatic` is `'new'`; `options` are the selectable types.
 */
export function effectiveTypeChoice(
  picked: TypeChoice | null,
  automatic: TypeChoice,
  options: readonly SurveyType[],
): TypeChoice {
  const rendered = (c: TypeChoice) => (c === 'new' ? automatic === 'new' : options.some((t) => t.id === c));
  if (picked !== null && rendered(picked)) return picked;
  return rendered(automatic) ? automatic : 'new';
}

export function newTypeLabel(className: string): string {
  const name = className.trim();
  return name ? `Ny type: ${name}` : 'Ny type (navngiv klassen)';
}

/**
 * The resource request. `maskRevision` is the run's `mask_revision`: the
 * server refuses (409) a mask that was overwritten since, e.g. by the label
 * queue, instead of filing pixels the user never saw.
 */
export function resourceRequest(
  maskLabel: number,
  className: string,
  choice: TypeChoice,
  maskRevision?: string | null,
): SegmentResourceRequest {
  const request: SegmentResourceRequest = { mask_label: maskLabel, class_name: className.trim() };
  if (choice !== 'new') request.type_id = choice;
  if (maskRevision) request.mask_revision = maskRevision;
  return request;
}

// ---------------------------------------------------------------- keys --

export type SegmentKeyAction = 'prev' | 'next' | 'mask' | 'run';

/**
 * The view's keys: ←/→ step frames, M toggles the mask overlay, Ctrl/⌘+Enter
 * runs (also from a prompt field). Nothing else fires while typing.
 */
export function segmentKeyAction(k: {
  key: string;
  inField: boolean;
  metaKey?: boolean;
  ctrlKey?: boolean;
  altKey?: boolean;
}): SegmentKeyAction | null {
  if (k.key === 'Enter' && (k.metaKey || k.ctrlKey)) return 'run';
  if (k.inField || k.metaKey || k.ctrlKey || k.altKey) return null;
  switch (k.key) {
    case 'ArrowLeft':
      return 'prev';
    case 'ArrowRight':
      return 'next';
    case 'm':
    case 'M':
      return 'mask';
    default:
      return null;
  }
}

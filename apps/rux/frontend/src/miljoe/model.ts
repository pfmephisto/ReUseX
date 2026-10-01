// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Miljø & prøver as data: the stage chain a card draws, what its action row
 * offers, the exact patches it sends, link toggling, and the approval-gate
 * feedback after a change. Pure, so the sample flow is testable without a DOM.
 *
 * The backend rule (core::update_sample_checked): a result can only be set
 * once the stage is `svar`, checked on the merged state — so recording a
 * result sends stage and result together, and undoing goes back to `sendt`
 * (a `svar` sample with no result counts as clean in environment_status()).
 *
 * Pending counts (the badge, `SurveySummary.pending_samples`) come only from
 * the server — there is no client-side equivalent here.
 */

import type { Sample, SampleCreate, SamplePatch, SampleResult, SampleStage, SurveyType } from '../api/types';
import type { TargetKind } from '../app/keyTargets';
import { RESULT_LABEL, STAGE_LABEL, type Tone } from '../kortlaegning/vocab';

export const STAGES: readonly SampleStage[] = ['planlagt', 'udtaget', 'sendt', 'svar'];

export type StepState = 'done' | 'current' | 'todo';

export interface ChainStep {
  stage: SampleStage;
  label: string;
  state: StepState;
}

export function chainSteps(s: Pick<Sample, 'stage'>): ChainStep[] {
  const at = STAGES.indexOf(s.stage);
  return STAGES.map((stage, i) => ({
    stage,
    label: STAGE_LABEL[stage],
    state: i < at ? 'done' : i === at ? 'current' : 'todo',
  }));
}

export function nextStage(stage: SampleStage): SampleStage | null {
  const i = STAGES.indexOf(stage);
  return i >= 0 && i < STAGES.length - 1 ? STAGES[i + 1] : null;
}

export function statusPill(s: Pick<Sample, 'stage' | 'result'>): { label: string; tone: Tone } {
  if (s.result === 'forurenet') return { label: RESULT_LABEL.forurenet, tone: 'crit' };
  if (s.result === 'ren') return { label: RESULT_LABEL.ren, tone: 'good' };
  if (s.stage === 'svar') return { label: STAGE_LABEL.svar, tone: 'accent' };
  return { label: STAGE_LABEL[s.stage], tone: 'wait' };
}

export type CardAction = 'advance' | 'answer' | 'answered';

/**
 * `advance` (Næste trin →) before the lab has the sample; `answer` (the two
 * result buttons) once it is sent — or answered without a result, which only
 * the API can produce; `answered` (the note) once a result exists.
 */
export function cardAction(s: Pick<Sample, 'stage' | 'result'>): CardAction {
  if (s.result !== null) return 'answered';
  if (s.stage === 'sendt' || s.stage === 'svar') return 'answer';
  return 'advance';
}

/** Only before the lab has the sample (`cardAction` would be `advance`) — never sends a bare `svar`. */
export function advancePatch(s: Pick<Sample, 'stage'>): SamplePatch | null {
  if (s.stage !== 'planlagt' && s.stage !== 'udtaget') return null;
  const next = nextStage(s.stage);
  return next ? { stage: next } : null;
}

/** One PATCH: the result is legal because the merged stage is `svar`. */
export function resultPatch(result: SampleResult): SamplePatch {
  return { stage: 'svar', result };
}

/** Back to awaiting the lab — never `svar` without a result (that counts as clean). */
export const UNDO_RESULT_PATCH: SamplePatch = { stage: 'sendt', result: null };

/** Takes the links as shown (a pending link draft, else the server's), hence `readonly`. */
export function answeredNote(s: { type_ids: readonly number[] }): string {
  const n = s.type_ids.length;
  return n === 0
    ? 'Svar registreret — prøven er ikke koblet til nogen type.'
    : `Svar registreret — miljøstatus opdateret på ${n} type(r) i kortlægningen.`;
}

export function linkedTypes(ids: readonly number[], types: SurveyType[]): SurveyType[] {
  const byId = new Map(types.map((t) => [t.id, t]));
  return ids.map((id) => byId.get(id)).filter((t): t is SurveyType => t !== undefined);
}

/** What the link picker offers: every non-rejected type, plus rejected ones still linked (so they can be unlinked). */
export function linkableTypes(types: SurveyType[], linked: readonly number[]): SurveyType[] {
  return types
    .filter((t) => t.review_status !== 'rejected' || linked.includes(t.id))
    .sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

export function toggleLink(ids: readonly number[], typeId: number): number[] {
  const next = ids.includes(typeId) ? ids.filter((id) => id !== typeId) : [...ids, typeId];
  return [...new Set(next)].sort((a, b) => a - b);
}

export function replaceSample(list: Sample[], updated: Sample): Sample[] {
  return list.map((s) => (s.id === updated.id ? updated : s));
}

export function addSample(list: Sample[], created: Sample): Sample[] {
  return [...list.filter((s) => s.id !== created.id), created].sort((a, b) => a.id - b.id);
}

export function removeSample(list: Sample[], id: number): Sample[] {
  return list.filter((s) => s.id !== id);
}

/**
 * The value a text draft commits on blur, or null to send nothing: unchanged
 * after trimming (an untouched blur), or emptied when the field is required.
 */
export function textCommit(draft: string, current: string, required: boolean): string | null {
  const value = draft.trim();
  if (value === current.trim()) return null;
  if (required && value === '') return null;
  return value;
}

export function createBody(title: string, what: string, typeIds: readonly number[]): SampleCreate | null {
  const t = title.trim();
  if (t === '') return null;
  return { title: t, what: what.trim(), type_ids: [...typeIds].sort((a, b) => a - b) };
}

export interface GateChange {
  /** Queued types that were awaiting a sample and now can be approved. */
  unblocked: SurveyType[];
  /** Types that just became forurenet. */
  contaminated: SurveyType[];
  /** Queued types that now await a sample. */
  blocked: SurveyType[];
  /** Approved types that now await a sample — approved, but blocking Indberetning. */
  reblocked: SurveyType[];
  /** Approved types that no longer await a sample — were blocking Indberetning, now don't. */
  released: SurveyType[];
}

/** Compares two server snapshots of the survey; never derives a status itself. Rejected types are ignored. */
export function gateChanges(before: SurveyType[], after: SurveyType[]): GateChange {
  const was = new Map(before.map((t) => [t.id, t.environment_status]));
  const out: GateChange = { unblocked: [], contaminated: [], blocked: [], reblocked: [], released: [] };
  for (const t of after) {
    const prev = was.get(t.id);
    if (prev === undefined || prev === t.environment_status || t.review_status === 'rejected') continue;
    if (t.environment_status === 'forurenet') out.contaminated.push(t);
    else if (t.environment_status === 'afventer') (t.review_status === 'approved' ? out.reblocked : out.blocked).push(t);
    else if (prev === 'afventer' && t.review_status === 'queue') out.unblocked.push(t);
    else if (prev === 'afventer' && t.review_status === 'approved') out.released.push(t);
  }
  return out;
}

function names(types: SurveyType[]): string {
  return types.map((t) => t.name).join(' · ');
}

export function gateMessage(code: string, c: GateChange): string | null {
  const parts: string[] = [];
  if (c.unblocked.length > 0) {
    const n = c.unblocked.length;
    parts.push(`${n === 1 ? '1 type kan' : `${n} typer kan`} nu godkendes (${names(c.unblocked)})`);
  }
  if (c.contaminated.length > 0) {
    // The adjective agrees in number: "A er nu forurenet", "A · B er nu forurenede".
    parts.push(`${names(c.contaminated)} er nu ${c.contaminated.length === 1 ? 'forurenet' : 'forurenede'}`);
  }
  if (c.blocked.length > 0) parts.push(`${names(c.blocked)} afventer nu prøvesvar`);
  if (c.reblocked.length > 0) parts.push(`${names(c.reblocked)} er godkendt, men afventer nu prøvesvar`);
  if (c.released.length > 0) parts.push(`${names(c.released)} blokerer ikke længere Indberetning`);
  return parts.length > 0 ? `${code}: ${parts.join('; ')}` : null;
}

export function resultToast(code: string, result: SampleResult): string {
  return `✓ ${code} · svar registreret: ${RESULT_LABEL[result]}`;
}

/**
 * Counts the links passed in — the card passes the draft links its editor
 * shows, not the server's, so the prompt matches what the user sees.
 */
export function deleteConfirmText(s: Pick<Sample, 'code' | 'title'> & { type_ids: readonly number[] }): string {
  const n = s.type_ids.length;
  if (n === 0) return `Slet ${s.code} · ${s.title}?`;
  const who = n === 1 ? '1 koblet type mister prøven, og dens' : `${n} koblede typer mister prøven, og deres`;
  return `Slet ${s.code} · ${s.title}? ${who} miljøstatus beregnes igen.`;
}

export type EditorKey = 'revert' | 'close' | 'commit' | 'submit';

/**
 * Keys inside a sample editor or the create form. Esc in a text field drops
 * that field's draft (and must not commit it); Esc anywhere else closes.
 * Enter in a single-line text field commits it; Ctrl/⌘+Enter submits from
 * anywhere. Enter/Space on buttons, links and checkboxes stay native.
 */
export function editorKeyAction(k: {
  key: string;
  kind: TargetKind;
  ctrlKey?: boolean;
  metaKey?: boolean;
  altKey?: boolean;
}): EditorKey | null {
  if (k.key === 'Escape') return k.kind === 'text' ? 'revert' : 'close';
  if (k.key === 'Enter' && (k.ctrlKey || k.metaKey)) return 'submit';
  if (k.key === 'Enter' && k.kind === 'text' && !k.altKey) return 'commit';
  return null;
}

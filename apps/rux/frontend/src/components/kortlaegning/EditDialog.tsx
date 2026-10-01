// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's edit dialog: a modal over the survey table for working a
 * survey type (or one of its building parts) end to end — parts, mængde,
 * behandling, note, photos and the evidence stage — then "Godkend & næste".
 *
 * Like `DetailPanel`, everything that decides *what* is shown is a pure,
 * exported function (`titlePrefix`, `partChips`, `quantityLabel`,
 * `partCountText`, `primaryLabel`, `primaryDisabled`, `photoStrip`,
 * `wrapFocusIndex`, `isTabbable`) so it is
 * unit-testable without a DOM. The review/approval rules themselves are
 * DetailPanel's (`approveBlocked`, `gateNoteText`, `sampleLineText`,
 * `quantityCommitValue`) and are reused, not re-derived.
 *
 * Keyboard: the page owns the key map (`onKeyDown`, see `dialogAction` in
 * `kortlaegning/keys.ts`) and restoring focus on close. The dialog itself
 * traps Tab, focuses the quantity field on open and after every move, and —
 * before handing a navigating key (PgUp/PgDn, ⌘/Ctrl+Enter, Esc) to the page
 * — blurs the focused field so its pending draft commits first instead of
 * being dropped by the selection change.
 */

import { useEffect, useId, useRef, useState } from 'react';
import type { KeyboardEvent } from 'react';

import { api } from '../../api/client';
import type { Sample, SurveyPart, SurveyType, Treatment, VisibleFrame } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import { useAsync } from '../../app/useAsync';
import type { EvidenceTab } from '../../kortlaegning/keys';
import { dialogAction } from '../../kortlaegning/keys';
import {
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatNumber,
  formatQuantity,
  formatTonnes,
  TREATMENT_LABEL,
} from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { Kbd } from '../Kbd';
import { Pill } from '../Pill';
import { approveBlocked, gateNoteText, quantityCommitValue, sampleLineText } from './DetailPanel';
import styles from './EditDialog.module.css';
import type { FrameLookup } from './EvidencePanel';
import { EvidencePanel, hasInstanceLink, instanceKey, resolveHighlightPart } from './EvidencePanel';

export interface EditDialogProps {
  type: SurveyType;
  part: SurveyPart | null;
  samples: Sample[];
  tab: EvidenceTab;
  onTab: (t: EvidenceTab) => void;
  /** A request is in flight — disables the buttons (never the field commits). */
  busy: boolean;
  /** Part chips; `null` = "Alle" (the type itself). */
  onSelectPart: (code: string | null) => void;
  onPrev: () => void;
  onNext: () => void;
  onClose: () => void;
  onApproveNext: () => void;
  onReject: () => void;
  /** type → redistributes across its parts; part → that part only. */
  onQuantity: (q: number) => void;
  onTreatment: (t: Treatment) => void;
  /** Committed on blur when changed. */
  onNote: (n: string) => void;
  onStar: () => void;
  /** The page's key map; runs after the dialog's own Tab trap. */
  onKeyDown: (e: KeyboardEvent) => void;
}

/** Thumbnails shown in the Fotos row before collapsing the rest into `+n`. */
export const PHOTO_STRIP_MAX = 5;

/** The head title's `RX-### · ` prefix when a part is selected, else ''. */
export function titlePrefix(part: SurveyPart | null): string {
  return part ? `${part.code} · ` : '';
}

export interface PartChip {
  /** `null` for the "Alle" chip. */
  code: string | null;
  label: string;
}

/** `Alle · {total} {unit}`, then `RX-### · room · {qty}` per part. */
export function partChips(type: SurveyType): PartChip[] {
  return [
    { code: null, label: `Alle · ${formatQuantity(type.quantity, type.unit)}` },
    ...type.parts.map((p) => ({
      code: p.code,
      label: `${p.code} · ${p.room_name} · ${formatNumber(p.quantity)}`,
    })),
  ];
}

/** The mængde field's label: a part's own quantity, or the type's aggregate. */
export function quantityLabel(part: SurveyPart | null): string {
  return part ? 'Mængde (denne del)' : 'Mængde (aggregeret — fordeles på delene)';
}

/** `n dele` (or `1 del`) after the confidence bar. */
export function partCountText(n: number): string {
  return `${n} ${n === 1 ? 'del' : 'dele'}`;
}

/** The primary foot button: an already-approved type just moves on. */
export function primaryLabel(type: SurveyType): string {
  return type.review_status === 'approved' ? 'Godkendt ✓ — næste' : 'Godkend & næste ✓';
}

/**
 * Whether the primary button is disabled. An already-approved type's
 * `Godkendt ✓ — næste` only advances, so `afventer` does not block it. The
 * page reuses this for the ⌘/Ctrl+Enter path.
 */
export function primaryDisabled(type: SurveyType, busy: boolean): boolean {
  return busy || (approveBlocked(type) && type.review_status !== 'approved');
}

/** Splits a frame list into the thumbnails shown and the `+n` overflow count. */
export function photoStrip(
  frames: readonly VisibleFrame[],
  max: number = PHOTO_STRIP_MAX,
): { visible: VisibleFrame[]; overflow: number } {
  return { visible: frames.slice(0, max), overflow: Math.max(0, frames.length - max) };
}

/**
 * The Tab trap: given the focusable count, the index of the focused element
 * (`-1` when focus is outside the list) and the Tab direction, returns the
 * index to move focus to — or `null` to let the browser's own Tab order
 * proceed (anywhere strictly inside the list).
 */
export function wrapFocusIndex(count: number, current: number, shift: boolean): number | null {
  if (count === 0) return null;
  if (current < 0) return shift ? count - 1 : 0;
  if (shift && current === 0) return count - 1;
  if (!shift && current === count - 1) return 0;
  return null;
}

const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), ' +
  'textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

/** The bits of an element `isTabbable` looks at; a structural type so tests can stub it. */
export interface TabbableProbe {
  /** `HTMLElement.hidden` is `boolean | "until-found"`; either truthy form hides. */
  hidden: boolean | string;
  getClientRects(): { length: number };
  closest(selectors: string): unknown;
}

/**
 * Whether a `FOCUSABLE` match can actually take Tab focus: not `hidden`,
 * rendered (has a layout box — `display: none` ancestors give none), and not
 * inside an `inert` subtree or a disabled fieldset.
 */
export function isTabbable(el: TabbableProbe): boolean {
  return !el.hidden && el.getClientRects().length > 0 && !el.closest('[inert],fieldset[disabled]');
}

function isField(el: Element | null): el is HTMLElement {
  return el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement || el instanceof HTMLSelectElement;
}

/** One Fotos thumbnail; a sunken placeholder when the image fails to load. */
function PhotoThumb({ frameId }: { frameId: number }) {
  const [errored, setErrored] = useState(false);
  if (errored) {
    return (
      <span className={styles.photoFallback} title={`Ramme ${frameId} kunne ikke hentes`}>
        Intet billede
      </span>
    );
  }
  return (
    <img
      className={styles.photo}
      src={api.frameImageUrl(frameId, 'color', { maxSize: 160 })}
      alt={`Ramme ${frameId}`}
      loading="lazy"
      onError={() => setErrored(true)}
    />
  );
}

export function EditDialog(props: EditDialogProps) {
  const { type, part, samples, tab, onTab, busy } = props;
  // The entity whose quantity/note/star this dialog edits: the selected part
  // when one is selected, otherwise the type itself.
  const current = part ?? type;
  const titleId = useId();
  const rootRef = useRef<HTMLDivElement>(null);

  const [quantityDraft, setQuantityDraft] = useState(() => formatNumber(current.quantity));
  const [noteDraft, setNoteDraft] = useState(() => current.note);
  const quantityInputRef = useRef<HTMLInputElement>(null);
  // Whether the quantity input is focused, so the server-resync effect below
  // never clobbers what the user is mid-typing.
  const quantityFocusedRef = useRef(false);
  // Bumped on every selection change; focusing in its own effect means the
  // field is selected *after* the render that put the new draft in it.
  const [focusTick, setFocusTick] = useState(0);

  // Reset the drafts on open and on every selection change, then ask for the
  // quantity field to be focused.
  useEffect(() => {
    setQuantityDraft(formatNumber(current.quantity));
    setNoteDraft(current.note);
    setFocusTick((t) => t + 1);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [type.id, part?.code]);

  useEffect(() => {
    if (focusTick === 0) return;
    const input = quantityInputRef.current;
    input?.focus();
    input?.select();
  }, [focusTick]);

  // A button that turns `disabled` while focused (Afvis / ☆ during a request)
  // drops focus to <body>, which would take the page's key map and the Tab
  // trap with it. On every busy transition, pull focus back to the dialog
  // root if it has left the dialog or sits on a now-disabled control.
  useEffect(() => {
    const root = rootRef.current;
    if (!root) return;
    const active = document.activeElement;
    const lost = !active || !root.contains(active);
    const onDisabled = active instanceof HTMLButtonElement && active.disabled;
    if (lost || onDisabled) root.focus();
  }, [busy]);

  // Re-sync the quantity draft when the server value changes under us (e.g.
  // a redistribution changes this part's share) — never while focused.
  useEffect(() => {
    if (!quantityFocusedRef.current) setQuantityDraft(formatNumber(current.quantity));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [current.quantity]);

  // Fotos row: the selected part's frames, or the type's first linked part's.
  const highlight = resolveHighlightPart(type, part);
  const linked = hasInstanceLink(highlight);
  const currentKey = linked ? instanceKey(highlight.cloud, highlight.instance_id) : null;
  const frames = useAsync<FrameLookup>(
    async (signal) => {
      if (!linked) return { key: '', frames: [], failed: false };
      const key = instanceKey(highlight.cloud, highlight.instance_id);
      try {
        const result = await api.instanceFrames(highlight.cloud, highlight.instance_id, signal);
        return { key, frames: result, failed: false };
      } catch (cause) {
        if (signal.aborted) throw cause;
        return { key, frames: [], failed: true };
      }
    },
    [highlight?.cloud, highlight?.instance_id],
  );
  // `useAsync` keeps the previous highlight's result around until the new
  // request settles; only a key match counts (see EvidencePanel's FrameLookup).
  const lookup = frames.data && frames.data.key === currentKey ? frames.data : undefined;

  function commitQuantity() {
    const value = quantityCommitValue(quantityDraft, current.quantity);
    if (value !== null) {
      props.onQuantity(value);
    } else {
      setQuantityDraft(formatNumber(current.quantity));
    }
  }

  function commitNote() {
    if (noteDraft !== current.note) props.onNote(noteDraft);
  }

  function handleKeyDown(e: KeyboardEvent<HTMLDivElement>) {
    const root = rootRef.current;
    if (e.key === 'Tab' && root) {
      const focusables = Array.from(root.querySelectorAll<HTMLElement>(FOCUSABLE)).filter(isTabbable);
      const index = focusables.indexOf(document.activeElement as HTMLElement);
      const next = wrapFocusIndex(focusables.length, index, e.shiftKey);
      if (next !== null) {
        e.preventDefault();
        focusables[next].focus();
      }
    }
    // A key that navigates away (move / approve & next / close) while a field
    // holds a draft: blur first so the draft commits against the current
    // selection rather than being reset by the next one.
    const active = document.activeElement;
    if (
      isField(active) &&
      root?.contains(active) &&
      dialogAction({ key: e.key, metaKey: e.metaKey, ctrlKey: e.ctrlKey, altKey: e.altKey, inField: true })
    ) {
      active.blur();
    }
    props.onKeyDown(e);
  }

  const chips = partChips(type);
  const queued = type.review_status === 'queue';
  const gateNote = queued ? gateNoteText(type, samples) : null;
  const tonnes = part ? '' : formatTonnes(type.mass_t);
  const strip = lookup && !lookup.failed ? photoStrip(lookup.frames) : null;
  const photoCount = lookup && !lookup.failed ? lookup.frames.length : null;

  let photoMessage: string | null = null;
  if (!linked) photoMessage = 'Ingen fotos — bygningsdelen er ikke koblet til en instans.';
  else if (!lookup) photoMessage = 'Indlæser fotos…';
  else if (lookup.failed) photoMessage = 'Fotos kunne ikke hentes.';
  else if (lookup.frames.length === 0) photoMessage = 'Ingen fotos fundet for denne instans.';

  return (
    <div className={styles.scrim}>
      <div
        ref={rootRef}
        // Focusable but not in the Tab order (FOCUSABLE skips tabindex=-1):
        // a click on a non-focusable area focuses the dialog itself, so the
        // keys keep reaching `handleKeyDown` instead of falling to <body>.
        tabIndex={-1}
        className={styles.dialog}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        onKeyDown={handleKeyDown}
      >
        <header className={styles.head}>
          <h2 id={titleId} className={styles.title}>
            {titlePrefix(part)}
            {type.name}
          </h2>
          {type.bim7aa_code && <Pill tone="accent">{type.bim7aa_code}</Pill>}
          <Pill tone={ENV_TONE[type.environment_status]}>{ENV_LABEL[type.environment_status]}</Pill>
          {current.starred && <Pill tone="warn">★ Vigtig</Pill>}
          <div className={styles.spacer} />
          <button
            type="button"
            className={styles.chromeButton}
            title="Forrige række (PgUp)"
            aria-label="Forrige række (PgUp)"
            onClick={props.onPrev}
          >
            ‹
          </button>
          <button
            type="button"
            className={styles.chromeButton}
            title="Næste række (PgDn)"
            aria-label="Næste række (PgDn)"
            onClick={props.onNext}
          >
            ›
          </button>
          <button
            type="button"
            className={styles.chromeButton}
            title="Luk (Esc)"
            aria-label="Luk (Esc)"
            onClick={props.onClose}
          >
            ✕
          </button>
        </header>

        <div className={styles.body}>
          <div className={styles.form}>
            <div className={styles.field}>
              <span className={styles.label}>Bygningsdele ({type.parts.length})</span>
              <div className={styles.chips}>
                {chips.map((chip) => {
                  const active = (part?.code ?? null) === chip.code;
                  return (
                    <button
                      key={chip.code ?? '*'}
                      type="button"
                      className={styles.chip}
                      data-active={active || undefined}
                      aria-pressed={active}
                      onClick={() => props.onSelectPart(chip.code)}
                    >
                      {chip.label}
                    </button>
                  );
                })}
              </div>
            </div>

            <div className={styles.field}>
              <label className={styles.label} htmlFor={`${titleId}-qty`}>
                {quantityLabel(part)}
              </label>
              <div className={styles.quantityRow}>
                <input
                  id={`${titleId}-qty`}
                  ref={quantityInputRef}
                  type="text"
                  inputMode="decimal"
                  className={`${styles.input} mono`}
                  value={quantityDraft}
                  onChange={(e) => setQuantityDraft(e.target.value)}
                  onFocus={() => {
                    quantityFocusedRef.current = true;
                  }}
                  onBlur={() => {
                    quantityFocusedRef.current = false;
                    commitQuantity();
                  }}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter' && !e.metaKey && !e.ctrlKey) quantityInputRef.current?.blur();
                  }}
                />
                <span className={styles.unit}>{type.unit}</span>
                {tonnes && <span className={`${styles.tonnes} mono`}>{tonnes}</span>}
              </div>
            </div>

            <div className={styles.field}>
              <span className={styles.label}>EAK-kode</span>
              <span className={styles.value}>
                <span className="mono">{type.eak_code}</span> · {type.eak_name}
              </span>
            </div>

            <div className={styles.field}>
              <label className={styles.label} htmlFor={`${titleId}-treatment`}>
                Behandling
              </label>
              <select
                id={`${titleId}-treatment`}
                className={styles.select}
                value={type.treatment}
                onChange={(e) => props.onTreatment(e.target.value as Treatment)}
              >
                {TREATMENTS.map((t) => (
                  <option key={t} value={t}>
                    {TREATMENT_LABEL[t]}
                  </option>
                ))}
              </select>
            </div>

            <div className={styles.field}>
              <span className={styles.label}>Sikkerhed (AI)</span>
              <span className={styles.confidenceRow}>
                <ConfidenceBar percent={confidencePercent(type.confidence)} />
                <span className={styles.value}>· {partCountText(type.parts.length)}</span>
              </span>
            </div>

            <p className={styles.sampleLine}>{sampleLineText(type, samples)}</p>

            <div className={styles.field}>
              <label className={styles.label} htmlFor={`${titleId}-note`}>
                Proces / håndtering
              </label>
              <textarea
                id={`${titleId}-note`}
                className={styles.textarea}
                rows={3}
                value={noteDraft}
                onChange={(e) => setNoteDraft(e.target.value)}
                onBlur={commitNote}
              />
            </div>

            <div className={styles.field}>
              <span className={styles.label}>Fotos{photoCount !== null && ` (${photoCount})`}</span>
              {strip && strip.visible.length > 0 ? (
                <div className={styles.photos}>
                  {strip.visible.map((f) => (
                    <PhotoThumb key={f.frame_id} frameId={f.frame_id} />
                  ))}
                  {strip.overflow > 0 && <span className={styles.photoMore}>+{strip.overflow}</span>}
                </div>
              ) : (
                <span className={styles.photoEmpty}>{photoMessage}</span>
              )}
            </div>

            <div>
              <button type="button" className={styles.ghost} onClick={props.onStar} disabled={busy}>
                {current.starred ? '★ Fjern vigtig' : '☆ Markér vigtig'}
              </button>
            </div>
          </div>

          <div className={styles.evidence}>
            <EvidencePanel type={type} part={part} tab={tab} onTab={onTab} variant="stage" />
          </div>
        </div>

        <footer className={styles.foot}>
          <span className={styles.hints}>
            <Kbd>⌘/Ctrl</Kbd>+<Kbd>Enter</Kbd> godkend &amp; næste · <Kbd>PgUp</Kbd>
            <Kbd>PgDn</Kbd> skift række/del · <Kbd>1</Kbd>–<Kbd>4</Kbd> skift visning · <Kbd>Esc</Kbd> luk
          </span>
          <div className={styles.spacer} />
          {gateNote && <span className={styles.gateNote}>{gateNote}</span>}
          <button type="button" className={styles.ghost} onClick={props.onReject} disabled={busy}>
            Afvis
          </button>
          <button
            type="button"
            className={styles.primary}
            onClick={props.onApproveNext}
            disabled={primaryDisabled(type, busy)}
          >
            {primaryLabel(type)}
          </button>
        </footer>
      </div>
    </div>
  );
}

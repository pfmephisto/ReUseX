// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's edit dialog: a modal over the survey table for working a
 * survey type (or one of its building parts) end to end — parts, mængde,
 * behandling, note, photos and the evidence stage — then "Godkend & næste".
 *
 * Like `DetailPanel`, everything that decides *what* is shown is a pure,
 * exported function (`titlePrefix`, `partChips`, `partCountText`,
 * `primaryLabel`, `primaryDisabled`, `photoStrip`, `wrapFocusIndex`,
 * `isTabbable`) so it is unit-testable without a DOM. The review/approval
 * rules themselves are DetailPanel's (`approveBlocked`, `gateNoteText`), the
 * sample line is `SampleLine` (links to the sample in Miljø & prøver) and the
 * drafts are `useQuantityNoteDrafts` — reused, not re-derived. The sample
 * links are `a[href]`, so the Tab trap below includes them.
 *
 * Keyboard: the page owns the key map (`onKeyDown`, see `dialogAction` in
 * `kortlaegning/keys.ts`) and restoring focus on close. The dialog itself
 * traps Tab and focuses its own root on open and after every move, so the
 * advertised single-key shortcuts (1–5, G, A, V) work straight away; Tab from
 * the root goes to the quantity field. Before handing a navigating key
 * (PgUp/PgDn, ⌘/Ctrl+Enter, Esc) to the page it blurs the focused field, so
 * the pending draft commits first instead of being dropped by the selection
 * change.
 */

import { useEffect, useId, useRef, useState } from 'react';
import type { KeyboardEvent } from 'react';

import type { Sample, SurveyPart, SurveyType, Treatment } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import { useCanEdit } from '../../app/CaseRoleContext';
import { trapTab } from '../../app/focusTrap';
import type { EvidenceTab } from '../../kortlaegning/keys';
import { dialogAction, EVIDENCE_LAST_KEY } from '../../kortlaegning/keys';
import { partLabel } from '../../kortlaegning/model';
import {
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatNumber,
  formatQuantity,
  formatTonnes,
  quantityLabel,
  TREATMENT_LABEL,
} from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { Kbd } from '../Kbd';
import { Pill } from '../Pill';
import { approveBlocked, gateNoteText } from './DetailPanel';
import styles from './EditDialog.module.css';
import { EvidencePanel } from './EvidencePanel';
import { PhotoStrip } from './PhotoStrip';
import { SampleLine } from './SampleLine';
import { useQuantityNoteDrafts } from './useQuantityNoteDrafts';

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

// The Fotos strip's pure pieces live in `kortlaegning/photo` (PhotoStrip
// and DetailPanel share them); re-exported so existing imports keep working.
export { PHOTO_STRIP_MAX, photoStrip } from '../../kortlaegning/photo';

/** The head title's `RX-### · ` prefix when a part is selected, else ''. */
export function titlePrefix(part: SurveyPart | null): string {
  return part ? `${part.code} · ` : '';
}

export interface PartChip {
  /** `null` for the "Alle" chip. */
  code: string | null;
  label: string;
}

/** `Alle · {total} {unit}`, then `RX-### · room · {qty}` per part (see `partLabel`). */
export function partChips(type: SurveyType): PartChip[] {
  return [
    { code: null, label: `Alle · ${formatQuantity(type.quantity, type.unit)}` },
    ...type.parts.map((p) => ({
      code: p.code,
      label: `${partLabel(p)} · ${formatNumber(p.quantity)}`,
    })),
  ];
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

// The Tab trap lives in `app/focusTrap` (FormDialog shares it); re-exported
// so existing callers keep importing it here.
export { isTabbable, wrapFocusIndex, type TabbableProbe } from '../../app/focusTrap';

function isField(el: Element | null): el is HTMLElement {
  return el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement || el instanceof HTMLSelectElement;
}

export function EditDialog(props: EditDialogProps) {
  const { type, part, samples, tab, onTab, busy } = props;
  // The entity whose quantity/note/star this dialog edits: the selected part
  // when one is selected, otherwise the type itself.
  const current = part ?? type;
  // A viewer steps through the dialog read-only; the write actions are hidden.
  const canEdit = useCanEdit();
  const titleId = useId();
  const rootRef = useRef<HTMLDivElement>(null);

  const { quantityProps, noteProps } = useQuantityNoteDrafts(current, {
    onQuantity: props.onQuantity,
    onNote: props.onNote,
  });
  const quantityInputRef = useRef<HTMLInputElement>(null);

  // On open and after every move, focus the dialog itself (not a field), so
  // the single-key shortcuts reach the page's key map instead of typing into
  // the quantity input. Deferred by one render via the tick: focusing the
  // root blurs a focused field, and that blur must run after the drafts have
  // been reset for the new selection, not in the same effect flush.
  const [focusTick, setFocusTick] = useState(0);
  useEffect(() => {
    setFocusTick((t) => t + 1);
  }, [type.id, part?.code]);
  useEffect(() => {
    if (focusTick > 0) rootRef.current?.focus({ preventScroll: true });
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

  function handleKeyDown(e: KeyboardEvent<HTMLDivElement>) {
    const root = rootRef.current;
    trapTab(e, root, quantityInputRef.current);
    // A key that navigates away (move / approve & next / close) while a field
    // holds a draft: blur first so the draft commits against the current
    // selection rather than being reset by the next one.
    const active = document.activeElement;
    if (
      isField(active) &&
      root?.contains(active) &&
      dialogAction({ key: e.key, metaKey: e.metaKey, ctrlKey: e.ctrlKey, altKey: e.altKey, inField: true })
    ) {
      // Blur by focusing the root, so the draft commits but focus stays in the
      // dialog: a refused ⌘/Ctrl+Enter changes nothing that would refocus it.
      root?.focus();
    }
    props.onKeyDown(e);
  }

  const chips = partChips(type);
  const queued = type.review_status === 'queue';
  const gateNote = queued ? gateNoteText(type, samples) : null;
  const tonnes = part ? '' : formatTonnes(type.mass_t);

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
                  {...quantityProps}
                  readOnly={!canEdit}
                  onKeyDown={(e) => {
                    // Back to the dialog root, which blurs (and so commits) the field
                    // and keeps the shortcuts live; a bare blur would drop focus to <body>.
                    if (e.key === 'Enter' && !e.metaKey && !e.ctrlKey) rootRef.current?.focus();
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
                disabled={!canEdit}
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

            <SampleLine className={styles.sampleLine} type={type} samples={samples} />

            <div className={styles.field}>
              <label className={styles.label} htmlFor={`${titleId}-note`}>
                Note
              </label>
              <textarea
                id={`${titleId}-note`}
                className={styles.textarea}
                rows={3}
                {...noteProps}
                readOnly={!canEdit}
              />
            </div>

            <PhotoStrip type={type} part={part} fieldClassName={styles.field} labelClassName={styles.label} />

            {canEdit && (
              <div>
                <button type="button" className={styles.ghost} onClick={props.onStar} disabled={busy}>
                  {current.starred ? '★ Fjern vigtig' : '☆ Markér vigtig'}
                </button>
              </div>
            )}
          </div>

          <div className={styles.evidence}>
            <EvidencePanel type={type} part={part} tab={tab} onTab={onTab} variant="stage" />
          </div>
        </div>

        <footer className={styles.foot}>
          <span className={styles.hints}>
            <Kbd>⌘/Ctrl</Kbd>+<Kbd>Enter</Kbd> godkend &amp; næste · <Kbd>PgUp</Kbd>
            <Kbd>PgDn</Kbd> skift række/del · <Kbd>1</Kbd>–<Kbd>{EVIDENCE_LAST_KEY}</Kbd> skift visning · <Kbd>Tab</Kbd> rediger mængde · <Kbd>Esc</Kbd> luk
          </span>
          <div className={styles.spacer} />
          {canEdit && gateNote && <span className={styles.gateNote}>{gateNote}</span>}
          {canEdit && (
            <>
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
            </>
          )}
        </footer>
      </div>
    </div>
  );
}

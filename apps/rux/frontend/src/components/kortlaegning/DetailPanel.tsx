// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's right-column detail panel: the selected survey type or
 * building part, editable (mængde, behandling, note, ★) and — for a type —
 * the review actions (Godkend / Afvis / Genåbn).
 *
 * The pieces that decide *what* the panel shows are pure, exported functions
 * (`panelTitle`, `approveBlocked`, `gateNoteText`, and — from
 * `kortlaegning/samples.ts`, re-exported here — `linkedSamples`,
 * `pendingSampleList`, `sampleLineModel`, `sampleLineText`) so the
 * review/approval rules are unit-testable without a DOM; the component itself
 * only wires them to props. The sample line links to the sample in Miljø &
 * prøver (or, with none linked, to registering one) via `SampleLine`.
 * The quantity/note drafts are `useQuantityNoteDrafts`, shared with EditDialog.
 *
 * The page reuses one instance across selections; the drafts reset themselves
 * whenever the selected type or part changes (the hook's selection-keyed
 * effect), so nothing depends on the caller re-keying the panel.
 */

import type { KeyboardEvent } from 'react';

import { fieldKeyAction as sharedFieldKeyAction } from '../../app/editorKeys';
import type { Sample, SurveyPart, SurveyType, Treatment } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import { pendingSampleList } from '../../kortlaegning/samples';
import {
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  quantityLabel,
  TREATMENT_LABEL,
} from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { EmptyState } from '../EmptyState';
import { Pill } from '../Pill';
import styles from './DetailPanel.module.css';
import { SampleLine } from './SampleLine';
import { useQuantityNoteDrafts } from './useQuantityNoteDrafts';

// The sample-line functions live in `kortlaegning/samples.ts` (SampleLine
// uses them too); re-exported so existing callers keep importing them here.
export type { SampleLineModel } from '../../kortlaegning/samples';
export {
  linkedSamples,
  pendingSampleList,
  SAMPLE_LINE_LINKED_PREFIX,
  SAMPLE_LINE_NONE,
  sampleLineModel,
  sampleLineText,
} from '../../kortlaegning/samples';

export interface DetailPanelProps {
  type: SurveyType | null;
  part: SurveyPart | null;
  /** All samples; the panel picks the ones linked to `type.sample_ids`. */
  samples: Sample[];
  /** A request is in flight — blocks the review actions. */
  busy: boolean;
  /** type → redistributes across its parts; part → that part only. */
  onQuantity: (q: number) => void;
  onTreatment: (t: Treatment) => void;
  /** Committed on blur when changed. */
  onNote: (note: string) => void;
  onStar: () => void;
  onApprove: () => void;
  onReject: () => void;
  onReopen: () => void;
  /**
   * The user is done with a field: Enter in the quantity field, or Esc in any
   * field. The field has already been left: on Enter its draft committed, on
   * Esc it was dropped; the page puts focus back on the table.
   */
  onDone: () => void;
}

/** `RX-### · {type name}` for a part, or just the type name. */
export function panelTitle(type: SurveyType, part: SurveyPart | null): string {
  return part ? `${part.code} · ${type.name}` : type.name;
}

/** `environment_status === 'afventer'` blocks approval, regardless of `busy`. */
export function approveBlocked(type: SurveyType): boolean {
  return type.environment_status === 'afventer';
}

/**
 * The gate note shown under the actions when blocked, or `null` when not
 * blocked. Names only the *pending* linked samples (`stage !== 'svar'`) —
 * matching the backend's own `environment_status` rule
 * (`libs/reusex/src/core/survey.cpp`: a sample only keeps a type `afventer`
 * while its stage isn't `svar` yet). When none of the linked samples resolve
 * as pending (e.g. no samples are linked at all), the note drops the
 * parenthetical rather than showing an empty `()`.
 */
export function gateNoteText(type: SurveyType, samples: Sample[]): string | null {
  if (!approveBlocked(type)) return null;
  const names = pendingSampleList(type, samples);
  if (!names) return 'Kan ikke godkendes endnu — afventer prøvesvar.';
  return `Kan ikke godkendes endnu — afventer prøvesvar (${names}).`;
}

/**
 * What a key does in a detail-panel field (Phase 5 R10): Esc drops the
 * field's draft without committing, Enter in a single-line field commits it.
 * Either way focus goes back to the table ("Esc tilbage"). The mapping is the
 * shared `fieldKeyAction` from `app/editorKeys` (the one Miljø and Overblik
 * use); this only adapts it to the panel's single-line flag and also takes a
 * bare key name.
 */
export function fieldKeyAction(
  key: string | { key: string; ctrlKey?: boolean; metaKey?: boolean; altKey?: boolean },
  singleLine: boolean,
): 'revert' | 'commit' | null {
  return sharedFieldKeyAction(typeof key === 'string' ? { key } : key, !singleLine);
}

export function DetailPanel({
  type,
  part,
  samples,
  busy,
  onQuantity,
  onTreatment,
  onNote,
  onStar,
  onApprove,
  onReject,
  onReopen,
  onDone,
}: DetailPanelProps) {
  // The entity whose quantity/note/star this panel edits: the selected part
  // when one is selected, otherwise the type itself.
  const current = part ?? type;
  const { quantityProps, noteProps, revertQuantity, revertNote } = useQuantityNoteDrafts(current, { onQuantity, onNote });

  function onFieldKey(e: KeyboardEvent<HTMLElement>, singleLine: boolean, revert: ((el: HTMLElement) => void) | null) {
    const action = fieldKeyAction(e, singleLine);
    if (!action) return;
    e.preventDefault();
    if (action === 'revert' && revert) revert(e.currentTarget);
    else e.currentTarget.blur();
    onDone();
  }

  if (!type || !current) {
    return (
      <div className={styles.panel}>
        <EmptyState title="Vælg en type eller bygningsdel i tabellen." />
      </div>
    );
  }

  const gateNote = gateNoteText(type, samples);
  const blocked = approveBlocked(type);
  const queued = type.review_status === 'queue';

  return (
    <div className={styles.panel}>
      <div className={styles.head}>
        <span className={styles.title}>{panelTitle(type, part)}</span>
        {type.bim7aa_code && (
          <Pill tone="accent">{type.bim7aa_code}</Pill>
        )}
        <Pill tone={ENV_TONE[type.environment_status]}>{ENV_LABEL[type.environment_status]}</Pill>
        {current.starred && <Pill tone="warn">★ Vigtig</Pill>}
      </div>

      <div className={styles.grid}>
        <div className={styles.field}>
          <span className={styles.label}>{quantityLabel(part)}</span>
          <div className={styles.quantityRow}>
            <input
              type="text"
              inputMode="decimal"
              className={`${styles.input} mono`}
              aria-label={quantityLabel(part)}
              {...quantityProps}
              onKeyDown={(e) => onFieldKey(e, true, revertQuantity)}
            />
            <span className={styles.unit}>{type.unit}</span>
          </div>
        </div>

        <div className={styles.field}>
          <span className={styles.label}>EAK-kode</span>
          <span className={styles.value}>
            <span className="mono">{type.eak_code}</span> · {type.eak_name}
          </span>
        </div>

        <div className={styles.field}>
          <span className={styles.label}>Behandling</span>
          <select
            className={styles.select}
            value={type.treatment}
            onChange={(e) => onTreatment(e.target.value as Treatment)}
            onKeyDown={(e) => onFieldKey(e, false, null)}
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
          <ConfidenceBar percent={confidencePercent(type.confidence)} />
        </div>
      </div>

      <SampleLine className={styles.sampleLine} type={type} samples={samples} />

      <div className={styles.field}>
        <span className={styles.label}>Proces / håndtering</span>
        <textarea
          className={styles.textarea}
          rows={3}
          {...noteProps}
          onKeyDown={(e) => onFieldKey(e, false, revertNote)}
        />
      </div>

      <div className={styles.actions}>
        <button type="button" className={styles.ghost} onClick={onStar} disabled={busy}>
          {current.starred ? '★ Fjern vigtig' : '☆ Markér vigtig'}
        </button>
        <div className={styles.spacer} />
        {queued ? (
          <>
            <button type="button" className={styles.ghost} onClick={onReject} disabled={busy}>
              Afvis
            </button>
            <button
              type="button"
              className={styles.primary}
              onClick={onApprove}
              disabled={blocked || busy}
            >
              Godkend mængde ✓
            </button>
          </>
        ) : (
          <button type="button" className={styles.ghost} onClick={onReopen} disabled={busy}>
            Genåbn
          </button>
        )}
      </div>

      {queued && gateNote && <p className={styles.gateNote}>{gateNote}</p>}
    </div>
  );
}

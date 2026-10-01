// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's right-column detail panel: the selected survey type or
 * building part, editable (mængde, behandling, note, ★) and — for a type —
 * the review actions (Godkend / Afvis / Genåbn).
 *
 * The pieces that decide *what* the panel shows are pure, exported functions
 * (`panelTitle`, `linkedSamples`, `sampleLineText`, `approveBlocked`,
 * `gateNoteText`, `quantityCommitValue`) so the review/approval rules are
 * unit-testable without a DOM; the component itself only wires them to props
 * and local draft state.
 */

import { useEffect, useRef, useState } from 'react';

import type { Sample, SurveyPart, SurveyType, Treatment } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import {
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatNumber,
  parseDanishNumber,
  STAGE_LABEL,
  TREATMENT_LABEL,
} from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { EmptyState } from '../EmptyState';
import { Pill } from '../Pill';
import styles from './DetailPanel.module.css';

/**
 * Mount this with `key={part ? `p:${part.code}` : `t:${type?.id}`}` (or
 * equivalent) on the Kortlægning page so a new selection gets a fresh
 * component instance — that is the primary way the quantity/note drafts are
 * reset. The component also resets them itself in an effect keyed on
 * `type?.id` / `part?.code`, so it stays correct even if a caller reuses one
 * instance across selections.
 */
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
}

/** `RX-### · {type name}` for a part, or just the type name. */
export function panelTitle(type: SurveyType, part: SurveyPart | null): string {
  return part ? `${part.code} · ${type.name}` : type.name;
}

/** The samples a survey type's `sample_ids` names, in that order. */
export function linkedSamples(type: SurveyType, samples: Sample[]): Sample[] {
  const byId = new Map(samples.map((s) => [s.id, s]));
  return type.sample_ids.map((id) => byId.get(id)).filter((s): s is Sample => s !== undefined);
}

/**
 * The sample line under the EAK/behandling fields: which sample(s) drive the
 * type's miljøstatus, or the screening-only fallback when none are linked.
 */
export function sampleLineText(type: SurveyType, samples: Sample[]): string {
  const linked = linkedSamples(type, samples);
  if (linked.length === 0) {
    return 'Ingen prøve koblet — miljøstatus fra screening: ren.';
  }
  const parts = linked.map((s) => {
    const line = `${s.code} · ${s.title} — ${STAGE_LABEL[s.stage]}`;
    return s.result ? `${line} · ${s.result}` : line;
  });
  return `Miljøstatus styres af ${parts.join(', ')}`;
}

/** `environment_status === 'afventer'` blocks approval, regardless of `busy`. */
export function approveBlocked(type: SurveyType): boolean {
  return type.environment_status === 'afventer';
}

/** The gate note shown under the actions when blocked, or `null` when not. */
export function gateNoteText(type: SurveyType, samples: Sample[]): string | null {
  if (!approveBlocked(type)) return null;
  const codes = linkedSamples(type, samples)
    .map((s) => s.code)
    .join(', ');
  return `Kan ikke godkendes endnu — afventer prøvesvar (${codes}).`;
}

/**
 * Parses a quantity draft against the value currently on the server. Returns
 * the number to send to `onQuantity`, or `null` when nothing should be sent —
 * either the text doesn't parse, or it parses to the unchanged value. Either
 * way the caller reverts the draft to `formatNumber(current)`.
 */
export function quantityCommitValue(draftText: string, current: number): number | null {
  const parsed = parseDanishNumber(draftText);
  if (parsed === null || parsed === current) return null;
  return parsed;
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
}: DetailPanelProps) {
  // The entity whose quantity/note/star this panel edits: the selected part
  // when one is selected, otherwise the type itself.
  const current = part ?? type;

  const [quantityDraft, setQuantityDraft] = useState(() => (current ? formatNumber(current.quantity) : ''));
  const [noteDraft, setNoteDraft] = useState(() => current?.note ?? '');
  const quantityInputRef = useRef<HTMLInputElement>(null);

  // Defense in depth: reset the drafts on a selection change even if the
  // page reuses one instance instead of keying it by selection (see the
  // props doc above).
  useEffect(() => {
    setQuantityDraft(current ? formatNumber(current.quantity) : '');
    setNoteDraft(current?.note ?? '');
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [type?.id, part?.code]);

  if (!type || !current) {
    return (
      <div className={styles.panel}>
        <EmptyState title="Vælg en type eller bygningsdel i tabellen." />
      </div>
    );
  }

  function commitQuantity() {
    if (!current) return;
    const value = quantityCommitValue(quantityDraft, current.quantity);
    if (value !== null) {
      onQuantity(value);
    } else {
      setQuantityDraft(formatNumber(current.quantity));
    }
  }

  function commitNote() {
    if (!current) return;
    if (noteDraft !== current.note) onNote(noteDraft);
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
          <span className={styles.label}>{part ? 'Mængde (denne del)' : 'Mængde (aggregeret)'}</span>
          <div className={styles.quantityRow}>
            <input
              ref={quantityInputRef}
              type="text"
              inputMode="decimal"
              className={`${styles.input} mono`}
              value={quantityDraft}
              onChange={(e) => setQuantityDraft(e.target.value)}
              onBlur={commitQuantity}
              onKeyDown={(e) => {
                if (e.key === 'Enter') quantityInputRef.current?.blur();
              }}
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

      <p className={styles.sampleLine}>{sampleLineText(type, samples)}</p>

      <div className={styles.field}>
        <span className={styles.label}>Proces / håndtering</span>
        <textarea
          className={styles.textarea}
          rows={3}
          value={noteDraft}
          onChange={(e) => setNoteDraft(e.target.value)}
          onBlur={commitNote}
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

      {gateNote && <p className={styles.gateNote}>{gateNote}</p>}
    </div>
  );
}

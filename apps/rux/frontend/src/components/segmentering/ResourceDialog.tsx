// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import type { SegmentResourceRequest, SurveyType } from '../../api/types';
import {
  defaultTypeChoice,
  effectiveTypeChoice,
  newTypeLabel,
  resourceRequest,
  selectableTypes,
  type ResultClass,
  type TypeChoice,
} from '../../data/segmentView';
import { FormDialog } from '../kortlaegning/FormDialog';
import styles from '../kortlaegning/FormDialog.module.css';

export interface ResourceDialogProps {
  cls: ResultClass;
  /** Survey types; null while loading. */
  types: SurveyType[] | null;
  /** `labels` cloud definitions (id → name), to preselect the type the server would use. */
  labelNames: Record<string, string> | null;
  /** The run's `mask_revision`, echoed so the server refuses a mask overwritten since. */
  maskRevision: string | null;
  busy: boolean;
  error: string | null;
  onCancel: () => void;
  onSubmit: (request: SegmentResourceRequest) => void;
}

/**
 * "Opret ressource fra markering": file one prompt's pixels as a new instance
 * and survey part. The class name is prefilled from the prompt; the target is
 * an existing type or a new one named after the class. "Ny type" is offered
 * only when the server would really create one (see `defaultTypeChoice`).
 */
export function ResourceDialog({ cls, types, labelNames, maskRevision, busy, error, onCancel, onSubmit }: ResourceDialogProps) {
  const [className, setClassName] = useState(cls.className);
  const [picked, setPicked] = useState<TypeChoice | null>(null);
  const [localError, setLocalError] = useState<string | null>(null);

  const options = selectableTypes(types ?? []);
  const automatic = defaultTypeChoice(types ?? [], className, labelNames);
  // Until the user picks, follow the class name as they type it. Always a
  // rendered option, so the select shows exactly what is submitted.
  const choice: TypeChoice = effectiveTypeChoice(picked, automatic, options);

  function submit() {
    if (busy) return;
    if (!className.trim()) {
      setLocalError('Giv markeringen et klassenavn, fx “dør”.');
      return;
    }
    onSubmit(resourceRequest(cls.index, className, choice, maskRevision));
  }

  return (
    <FormDialog
      title="Opret ressource fra markering"
      onCancel={onCancel}
      onSubmit={submit}
      error={localError ?? error}
      actions={
        <>
          <button type="button" className={styles.btnGhost} onClick={onCancel}>
            Annuller
          </button>
          <button type="submit" className={styles.btnPrimary} disabled={busy || types === null}>
            {busy ? 'Opretter…' : 'Opret ressource'}
          </button>
        </>
      }
    >
      <p className={styles.note}>
        Markering {cls.index + 1}
        {cls.pixels !== null ? ` · ${cls.pixels.toLocaleString('da-DK')} px` : ''} placeres i punktskyen
        gennem billedets dybde og bliver en ny instans og en ny ressource.
      </p>
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Klasse</span>
        <input
          className={styles.input}
          value={className}
          placeholder="fx dør, vindue, radiator"
          onChange={(e) => {
            setClassName(e.target.value);
            setLocalError(null);
          }}
          autoFocus
        />
      </label>
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Type i Kortlægning</span>
        <select
          className={styles.input}
          value={String(choice)}
          disabled={types === null}
          onChange={(e) => setPicked(e.target.value === 'new' ? 'new' : Number(e.target.value))}
        >
          {automatic === 'new' && <option value="new">{newTypeLabel(className)}</option>}
          {options.map((t) => (
            <option key={t.id} value={t.id}>
              {t.name}
            </option>
          ))}
        </select>
      </label>
    </FormDialog>
  );
}

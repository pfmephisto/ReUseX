// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** "Tilføj ressource" (spec §6.1): a manual part under an existing type, with an optional name. */

import { useState } from 'react';

import type { ResourceCreate, SurveyType } from '../../api/types';
import { addableTypes } from '../../kortlaegning/resources';
import { FormDialog } from './FormDialog';
import styles from './FormDialog.module.css';

export interface AddResourceDialogProps {
  types: SurveyType[];
  /** The selected type, preselected. */
  defaultTypeId: number | null;
  busy: boolean;
  onCancel: () => void;
  onSubmit: (body: ResourceCreate) => void;
}

export function AddResourceDialog({ types, defaultTypeId, busy, onCancel, onSubmit }: AddResourceDialogProps) {
  const options = addableTypes(types);
  const [typeId, setTypeId] = useState<number | null>(
    options.some((t) => t.id === defaultTypeId) ? defaultTypeId : (options[0]?.id ?? null),
  );
  const [name, setName] = useState('');
  const [error, setError] = useState<string | null>(null);

  function submit() {
    if (typeId === null) {
      setError('Vælg en type.');
      return;
    }
    const trimmed = name.trim();
    onSubmit(trimmed ? { type_id: typeId, name: trimmed } : { type_id: typeId });
  }

  return (
    <FormDialog
      title="Tilføj ressource"
      onCancel={onCancel}
      onSubmit={submit}
      error={error}
      actions={
        <>
          <button type="button" className={styles.btnGhost} onClick={onCancel}>
            Annuller
          </button>
          <button type="submit" className={styles.btnPrimary} disabled={busy || typeId === null}>
            Tilføj ressource
          </button>
        </>
      }
    >
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Type</span>
        <select
          className={styles.input}
          value={typeId === null ? '' : String(typeId)}
          onChange={(e) => setTypeId(Number(e.target.value))}
          autoFocus
        >
          {options.map((t) => (
            <option key={t.id} value={t.id}>
              {t.name}
            </option>
          ))}
        </select>
      </label>
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Navn (valgfrit)</span>
        <input className={styles.input} value={name} onChange={(e) => setName(e.target.value)} />
      </label>
    </FormDialog>
  );
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * "Tilføj kolonne" (spec §6.1): create a user column and append it to the
 * selected template. On a seed template the dialog says so and offers
 * "Opret kopi af skabelonen i stedet" (plan R11). The draft is validated
 * here, before any request (`columnDraftError`); a name the server still
 * refuses (409, R3-D3) comes back as `serverError` and shows in the same
 * place.
 */

import { useEffect, useRef, useState } from 'react';

import type { Template } from '../../api/types';
import {
  COLUMN_KIND_LABEL,
  COLUMN_KINDS,
  columnDraftError,
  EMPTY_COLUMN_DRAFT,
  seedNote,
  type ColumnDraft,
  type ColumnKind,
} from '../../kortlaegning/columnDraft';
import { FormDialog } from './FormDialog';
import styles from './FormDialog.module.css';

export interface AddColumnDialogProps {
  template: Template | null;
  /** Every catalogue key's label: a new column may not reuse one. */
  existingLabels: string[];
  busy: boolean;
  /** The server's refusal of the last submit (a 409 name conflict), or null. */
  serverError: string | null;
  onCancel: () => void;
  onSubmit: (draft: ColumnDraft, copyInstead: boolean) => void;
}

export function AddColumnDialog({
  template,
  existingLabels,
  busy,
  serverError,
  onCancel,
  onSubmit,
}: AddColumnDialogProps) {
  const [draft, setDraft] = useState<ColumnDraft>(EMPTY_COLUMN_DRAFT);
  const [error, setError] = useState<string | null>(null);
  const note = seedNote(template);
  const nameRef = useRef<HTMLInputElement>(null);

  // A refused name is the thing to fix: put the caret back on it. (The
  // submit button was disabled while the request ran, which drops focus to
  // the body, where Esc would no longer reach the dialog.)
  useEffect(() => {
    if (serverError) nameRef.current?.select();
  }, [serverError]);

  function submit(copyInstead: boolean) {
    if (busy) return;
    const problem = columnDraftError(draft, existingLabels);
    setError(problem);
    if (!problem) onSubmit(draft, copyInstead);
  }

  return (
    <FormDialog
      title="Tilføj kolonne"
      onCancel={onCancel}
      onSubmit={() => submit(false)}
      error={error ?? serverError}
      actions={
        <>
          <button type="button" className={styles.btnGhost} onClick={onCancel}>
            Annuller
          </button>
          {note && (
            <button type="button" className={styles.btnGhost} disabled={busy} onClick={() => submit(true)}>
              Opret kopi af skabelonen i stedet
            </button>
          )}
          <button type="submit" className={styles.btnPrimary} disabled={busy || template === null}>
            Tilføj kolonne
          </button>
        </>
      }
    >
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Navn</span>
        <input
          ref={nameRef}
          className={styles.input}
          value={draft.name}
          onChange={(e) => setDraft({ ...draft, name: e.target.value })}
          autoFocus
        />
      </label>
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Type</span>
        <select
          className={styles.input}
          value={draft.kind}
          onChange={(e) => setDraft({ ...draft, kind: e.target.value as ColumnKind })}
        >
          {COLUMN_KINDS.map((k) => (
            <option key={k} value={k}>
              {COLUMN_KIND_LABEL[k]}
            </option>
          ))}
        </select>
      </label>
      {draft.kind === 'select' && (
        <label className={styles.field}>
          <span className={styles.fieldLabel}>Valgmuligheder (komma eller ny linje)</span>
          <textarea
            className={styles.input}
            rows={3}
            value={draft.optionsText}
            onChange={(e) => setDraft({ ...draft, optionsText: e.target.value })}
          />
        </label>
      )}
      {note && <p className={styles.note}>{note}</p>}
    </FormDialog>
  );
}

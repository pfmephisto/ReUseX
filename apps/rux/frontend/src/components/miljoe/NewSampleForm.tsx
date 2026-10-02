// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState } from 'react';

import type { SampleCreate, SurveyType } from '../../api/types';
import { formKeyDown } from '../../app/editorKeys';
import { createBody, toggleLink } from '../../miljoe/model';
import { LinkPicker } from './LinkPicker';
import styles from './NewSampleForm.module.css';

export interface NewSampleFormProps {
  types: SurveyType[];
  /** Pre-linked types (`/miljoe?ny=<typeId>`). */
  initialTypeIds: readonly number[];
  busy: boolean;
  onSubmit: (body: SampleCreate) => void;
  onCancel: () => void;
}

/**
 * `+ Ny prøve`: title, what was sampled, and the types it covers. The server
 * assigns the P-## code and starts the sample at Planlagt. Nothing is sent
 * until `Registrér prøve`; Esc cancels from anywhere in the form.
 */
export function NewSampleForm({ types, initialTypeIds, busy, onSubmit, onCancel }: NewSampleFormProps) {
  const [title, setTitle] = useState('');
  const [what, setWhat] = useState('');
  const [typeIds, setTypeIds] = useState<number[]>(() => [...initialTypeIds]);
  const body = createBody(title, what, typeIds);
  const id = useId();
  const headingId = `${id}-heading`;
  const titleId = `${id}-titel`;
  const whatId = `${id}-hvad`;
  const titleInput = useRef<HTMLInputElement>(null);

  // Opening the form lands on Titel.
  useEffect(() => {
    titleInput.current?.focus();
  }, []);

  function submit() {
    if (body && !busy) onSubmit(body);
  }


  return (
    <form
      className={styles.form}
      aria-labelledby={headingId}
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      onKeyDown={(e) => formKeyDown(e, { onCancel, onSubmit: submit })}
    >
      <h3 id={headingId} className={styles.title}>
        Ny prøve
      </h3>
      <div className={styles.fields}>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={titleId}>
            Titel
          </label>
          <input
            ref={titleInput}
            id={titleId}
            className={styles.input}
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            placeholder="fx PCB i fugemasse"
            required
          />
        </div>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={whatId}>
            Hvad er udtaget, og hvor
          </label>
          <input
            id={whatId}
            className={styles.input}
            value={what}
            onChange={(e) => setWhat(e.target.value)}
            placeholder="fx Fugemasse omkring vinduespartier, kontorområde"
          />
        </div>
      </div>
      <LinkPicker
        types={types}
        selected={typeIds}
        onToggle={(typeId) => setTypeIds((ids) => toggleLink(ids, typeId))}
      />
      <p className={styles.hint}>Koden (P-##) tildeles automatisk. Prøven starter som Planlagt.</p>
      <div className={styles.actions}>
        <button type="button" className={styles.btnGhost} onClick={onCancel}>
          Annullér
        </button>
        <button type="submit" className={styles.btnPrimary} disabled={busy || body === null}>
          Registrér prøve
        </button>
      </div>
    </form>
  );
}

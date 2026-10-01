// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState, type KeyboardEvent } from 'react';

import type { SampleCreate, SurveyType } from '../../api/types';
import { kindOf } from '../../app/keyTargets';
import { createBody, editorKeyAction, toggleLink } from '../../miljoe/model';
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

  function onKeyDown(e: KeyboardEvent<HTMLFormElement>) {
    const action = editorKeyAction({
      key: e.key,
      kind: kindOf(e.target),
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
      altKey: e.altKey,
    });
    // Unlike the sample editor, Esc in a text field ('revert') cancels the
    // whole form rather than reverting that field: nothing here is saved
    // yet, so there is no committed value to revert to.
    if (action === 'revert' || action === 'close') {
      e.preventDefault();
      e.stopPropagation(); // handled here: no page-level handler may act on it too
      onCancel();
    } else if (action === 'submit') {
      e.preventDefault();
      e.stopPropagation();
      submit();
    }
    // 'commit' (plain Enter in a text input) falls through to the native submit.
  }

  return (
    <form
      className={styles.form}
      aria-labelledby={headingId}
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      onKeyDown={onKeyDown}
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

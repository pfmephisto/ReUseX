// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState, type RefObject } from 'react';

import type { SampleCreate, SurveyPart } from '../../api/types';
import { formKeyDown } from '../../app/editorKeys';
import { createOnceGuard } from '../../app/onceGuard';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import { STAGE_LABEL } from '../../kortlaegning/vocab';
import { nextLabel, onsiteSampleBody, starButton, type Stop } from '../../onsite/model';
import styles from './CaptureSheet.module.css';

export interface CaptureSheetProps {
  part: SurveyPart;
  busy: boolean;
  next: Stop | null;
  /** Return the mutation's promise: the double-tap guard stays shut until it settles. */
  onStar: () => Promise<void> | void;
  onNote: (note: string) => void;
  /** Return the mutation's promise; call `done()` on success to close the form. */
  onRegister: (body: SampleCreate, done: () => void) => Promise<void> | void;
  onNext: () => void;
  /** Take focus once mounted (after Videre): the Videre button, or the sheet when there is no next part. */
  focusOnMount?: boolean;
}

interface SampleFormProps {
  part: SurveyPart;
  busy: boolean;
  formRef: RefObject<HTMLFormElement | null>;
  onSubmit: (body: SampleCreate) => Promise<void> | void;
  onCancel: () => void;
}

/**
 * `Registrér prøve her`, opened in the sheet. Nothing is sent until
 * `Registrér prøve`; Esc anywhere in the form cancels it (nothing in it is
 * saved yet), Ctrl/⌘+Enter submits — the keys of Miljø's create form.
 */
function SampleForm({ part, busy, formRef, onSubmit, onCancel }: SampleFormProps) {
  const [title, setTitle] = useState('');
  const [what, setWhat] = useState('');
  const body = onsiteSampleBody(title, what, part);
  const id = useId();
  const titleRef = useRef<HTMLInputElement>(null);
  const [guard] = useState(createOnceGuard);
  useEffect(() => {
    titleRef.current?.focus();
  }, []);

  function submit() {
    if (body && !busy) guard.run(() => onSubmit(body));
  }

  return (
    <form
      ref={formRef}
      className={styles.form}
      aria-label={`Ny prøve ved ${part.code}`}
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      // Esc cancels (nothing here is saved yet), Ctrl/⌘+Enter submits.
      onKeyDown={(e) => formKeyDown(e, { onCancel, onSubmit: submit })}
    >
      <div className={styles.field}>
        <label className={styles.label} htmlFor={`${id}-titel`}>
          Prøve
        </label>
        <input
          ref={titleRef}
          id={`${id}-titel`}
          className={styles.input}
          value={title}
          onChange={(e) => setTitle(e.target.value)}
          placeholder="fx PCB i fugemasse"
          required
        />
      </div>
      <div className={styles.field}>
        <label className={styles.label} htmlFor={`${id}-hvor`}>
          Hvor præcist
        </label>
        <input
          id={`${id}-hvor`}
          className={styles.input}
          value={what}
          onChange={(e) => setWhat(e.target.value)}
          placeholder="fx fuge ved vindue mod nord"
        />
      </div>
      <p className={styles.hint}>
        Kobles til {part.code} og dens type. Prøven registreres som {STAGE_LABEL.udtaget}.
      </p>
      <div className={styles.formActions}>
        <button type="button" className={styles.ghost} onClick={onCancel}>
          Annullér
        </button>
        <button type="submit" className={styles.primary} disabled={busy || body === null}>
          Registrér prøve
        </button>
      </div>
    </form>
  );
}

/**
 * The prototype's navy sheet: ★, a sample registered here, a quick note, and
 * Videre. Writes go against the part only (R7). The note commits on blur and
 * Enter, never gated on `busy`; Esc reverts it. ★ and the sample's submit are
 * buttons, gated on `busy` and guarded against a double tap. The page re-keys
 * this per part (R6), which resets the note draft and closes the form.
 */
export function CaptureSheet({
  part,
  busy,
  next,
  onStar,
  onNote,
  onRegister,
  onNext,
  focusOnMount = false,
}: CaptureSheetProps) {
  const home = useRef<HTMLDivElement>(null);
  const nextButton = useRef<HTMLButtonElement>(null);
  // Mount only: the page re-keys the sheet per part, so this runs once per part.
  const focusOnMountRef = useRef(focusOnMount);
  useEffect(() => {
    if (!focusOnMountRef.current) return;
    const button = nextButton.current;
    (button && !button.disabled ? button : home.current)?.focus();
  }, []);
  const note = useTextDraft(part.note, onNote);
  const [form, setForm] = useState(false);
  const star = starButton(part.starred);
  const [starGuard] = useState(createOnceGuard);
  const noteId = useId();
  const formRef = useRef<HTMLFormElement>(null);
  const opener = useRef<HTMLButtonElement>(null);
  const refocusOpener = useRef(false);

  // The opener only exists again after the form has unmounted.
  useEffect(() => {
    if (!form && refocusOpener.current) {
      refocusOpener.current = false;
      opener.current?.focus();
    }
  }, [form]);

  // Cancel: the user is in the form, so focus goes back to its opener.
  const cancelForm = () => {
    refocusOpener.current = true;
    setForm(false);
  };

  // Success arrives later: take focus back only if it is still in the form
  // (or fell to <body>), never from somewhere the user has moved on to.
  const doneForm = () => {
    const active = document.activeElement;
    refocusOpener.current = active === null || active === document.body || (formRef.current?.contains(active) ?? false);
    setForm(false);
  };

  return (
    <div ref={home} className={styles.sheet} tabIndex={-1}>
      <button
        type="button"
        className={styles.row}
        data-on={part.starred || undefined}
        aria-pressed={part.starred}
        disabled={busy}
        onClick={() => {
          if (!busy) starGuard.run(onStar);
        }}
      >
        <span className={styles.icon} aria-hidden="true">
          {star.icon}
        </span>
        {star.text}
      </button>

      {form ? (
        <SampleForm
          part={part}
          busy={busy}
          formRef={formRef}
          onSubmit={(body) => onRegister(body, doneForm)}
          onCancel={cancelForm}
        />
      ) : (
        <button ref={opener} type="button" className={styles.row} onClick={() => setForm(true)}>
          <span className={styles.icon} aria-hidden="true">
            ◎
          </span>
          Registrér prøve her
        </button>
      )}

      <label className={`${styles.row} ${styles.noteRow}`} htmlFor={noteId}>
        <span className={styles.icon} aria-hidden="true">
          ✎
        </span>
        <span className={styles.noteLabel}>Note</span>
        <input
          id={noteId}
          className={styles.noteInput}
          placeholder="Hurtig note…"
          {...note.props}
          onKeyDown={fieldKeys(note, home)}
        />
      </label>

      <button ref={nextButton} type="button" className={styles.next} disabled={next === null} onClick={onNext}>
        {nextLabel(next)}
      </button>
    </div>
  );
}

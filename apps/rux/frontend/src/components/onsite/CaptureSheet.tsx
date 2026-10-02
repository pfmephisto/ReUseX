// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState, type KeyboardEvent } from 'react';

import type { SampleCreate, SurveyPart } from '../../api/types';
import { editorKeyAction } from '../../app/editorKeys';
import { kindOf } from '../../app/keyTargets';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import { STAGE_LABEL } from '../../kortlaegning/vocab';
import { nextLabel, onsiteSampleBody, starButton, type Stop } from '../../onsite/model';
import styles from './CaptureSheet.module.css';

export interface CaptureSheetProps {
  part: SurveyPart;
  busy: boolean;
  next: Stop | null;
  onStar: () => void;
  onNote: (note: string) => void;
  onRegister: (body: SampleCreate, done: () => void) => void;
  onNext: () => void;
}

/**
 * A synchronous once-only guard for a busy-gated button (RapportPage's
 * generate guard): `busy` only disables the button on the next render, so a
 * fast double tap inside one frame would otherwise send twice. The guard
 * opens again once `busy` settles back to false (or a `reset` dep changes).
 */
function useOnceWhileBusy(busy: boolean, reset: unknown = null) {
  const inFlight = useRef(false);
  useEffect(() => {
    if (!busy) inFlight.current = false;
  }, [busy, reset]);
  return (run: () => void) => {
    if (inFlight.current || busy) return;
    inFlight.current = true;
    run();
  };
}

interface SampleFormProps {
  part: SurveyPart;
  busy: boolean;
  onSubmit: (body: SampleCreate) => void;
  onCancel: () => void;
}

/**
 * `Registrér prøve her`, opened in the sheet. Nothing is sent until
 * `Registrér prøve`; Esc anywhere in the form cancels it (nothing in it is
 * saved yet), Ctrl/⌘+Enter submits — the keys of Miljø's create form.
 */
function SampleForm({ part, busy, onSubmit, onCancel }: SampleFormProps) {
  const [title, setTitle] = useState('');
  const [what, setWhat] = useState('');
  const body = onsiteSampleBody(title, what, part);
  const id = useId();
  const titleRef = useRef<HTMLInputElement>(null);
  const once = useOnceWhileBusy(busy);
  useEffect(() => {
    titleRef.current?.focus();
  }, []);

  function submit() {
    if (body) once(() => onSubmit(body));
  }

  function onKeyDown(e: KeyboardEvent<HTMLFormElement>) {
    const action = editorKeyAction({
      key: e.key,
      kind: kindOf(e.target),
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
      altKey: e.altKey,
    });
    // As in Miljø's create form, Esc in a text field ('revert') cancels the
    // whole form: nothing here is saved yet, so there is nothing to revert to.
    if (action === 'revert' || action === 'close') {
      e.preventDefault();
      e.stopPropagation();
      onCancel();
    } else if (action === 'submit') {
      e.preventDefault();
      e.stopPropagation();
      submit();
    }
    // 'commit' (plain Enter in a field) falls through to the native submit.
  }

  return (
    <form
      className={styles.form}
      aria-label={`Ny prøve ved ${part.code}`}
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      onKeyDown={onKeyDown}
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
export function CaptureSheet({ part, busy, next, onStar, onNote, onRegister, onNext }: CaptureSheetProps) {
  const home = useRef<HTMLDivElement>(null);
  const note = useTextDraft(part.note, onNote);
  const [form, setForm] = useState(false);
  const star = starButton(part.starred);
  const starOnce = useOnceWhileBusy(busy, part.starred);
  const noteId = useId();

  const closeForm = () => {
    setForm(false);
    home.current?.focus(); // never drop focus to <body>
  };

  return (
    <div ref={home} className={styles.sheet} tabIndex={-1}>
      <button
        type="button"
        className={styles.row}
        data-on={part.starred || undefined}
        aria-pressed={part.starred}
        disabled={busy}
        onClick={() => starOnce(onStar)}
      >
        <span className={styles.icon} aria-hidden="true">
          {star.icon}
        </span>
        {star.text}
      </button>

      {form ? (
        <SampleForm part={part} busy={busy} onSubmit={(body) => onRegister(body, closeForm)} onCancel={closeForm} />
      ) : (
        <button type="button" className={styles.row} onClick={() => setForm(true)}>
          <span className={styles.icon} aria-hidden="true">
            ◎
          </span>
          Registrér prøve her
        </button>
      )}

      <div className={`${styles.row} ${styles.noteRow}`}>
        <span className={styles.icon} aria-hidden="true">
          ✎
        </span>
        <label className={styles.noteLabel} htmlFor={noteId}>
          Note
        </label>
        <input
          id={noteId}
          className={styles.noteInput}
          placeholder="Hurtig note…"
          {...note.props}
          onKeyDown={fieldKeys(note, home)}
        />
      </div>

      <button type="button" className={styles.next} disabled={next === null} onClick={onNext}>
        {nextLabel(next)}
      </button>
    </div>
  );
}

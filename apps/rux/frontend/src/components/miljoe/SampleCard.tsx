// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, type KeyboardEvent } from 'react';
import { Link } from 'react-router-dom';

import type { Sample, SampleResult, SurveyType } from '../../api/types';
import { kindOf } from '../../app/keyTargets';
import { surveyTypeHref } from '../../app/links';
import { STAGE_LABEL } from '../../kortlaegning/vocab';
import {
  answeredNote,
  cardAction,
  deleteConfirmText,
  editorKeyAction,
  linkedTypes,
  nextStage,
  statusPill,
} from '../../miljoe/model';
import { useTextDraft, type TextDraft } from '../../miljoe/useTextDraft';
import { Pill } from '../Pill';
import { LinkPicker } from './LinkPicker';
import { StageChain } from './StageChain';
import styles from './SampleCard.module.css';

export interface SampleCardProps {
  sample: Sample;
  types: SurveyType[];
  /** The linked type ids to show: the pending link draft, else the server's `type_ids`. */
  linkedIds: readonly number[];
  busy: boolean;
  /** Deep-linked (`?sample=`) or just created: highlighted. */
  focused: boolean;
  editing: boolean;
  onEditing: (open: boolean) => void;
  onAdvance: () => void;
  onResult: (result: SampleResult) => void;
  onUndoResult: () => void;
  onTitle: (title: string) => void;
  onWhat: (what: string) => void;
  onToggleLink: (typeId: number) => void;
  onDelete: () => void;
  cardRef?: (el: HTMLElement | null) => void;
}

/** Enter commits (blurs) a text field, Esc reverts it without committing. */
function fieldKeys(draft: TextDraft) {
  return (e: KeyboardEvent<HTMLInputElement>) => {
    const action = editorKeyAction({
      key: e.key,
      kind: 'text',
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
      altKey: e.altKey,
    });
    if (action === 'revert') {
      e.preventDefault();
      e.stopPropagation(); // the editor's Esc would otherwise close it
      draft.revert(e.currentTarget);
    } else if (action === 'commit') {
      e.preventDefault();
      e.currentTarget.blur();
    }
    // 'submit' (Ctrl/⌘+Enter) bubbles to the editor.
  };
}

/** "Koblet: A, B" — a rejected type is plain text, never a link (R8). */
function LinkedLine({ types }: { types: SurveyType[] }) {
  if (types.length === 0) return <>Ikke koblet til en type</>;
  return (
    <span>
      Koblet:{' '}
      {types.map((t, i) => (
        <span key={t.id}>
          {i > 0 && ', '}
          {t.review_status === 'rejected' ? (
            `${t.name} (afvist)`
          ) : (
            <Link className={styles.typeLink} to={surveyTypeHref(t.id)}>
              {t.name}
            </Link>
          )}
        </span>
      ))}
    </span>
  );
}

interface SampleEditorProps extends SampleCardProps {
  id: string;
  /** Close the editor and hand focus back to the card's Rediger button. */
  onClose: () => void;
}

function SampleEditor({
  sample,
  types,
  linkedIds,
  busy,
  onTitle,
  onWhat,
  onToggleLink,
  onDelete,
  onClose,
  id,
}: SampleEditorProps) {
  const title = useTextDraft(sample.title, onTitle, true);
  const what = useTextDraft(sample.what, onWhat);
  const titleId = `sample-${sample.id}-edit-title`;
  const whatId = `sample-${sample.id}-edit-what`;
  const titleInput = useRef<HTMLInputElement>(null);

  // Opening the editor lands on Titel.
  useEffect(() => {
    titleInput.current?.focus();
  }, []);

  // Esc outside a text field closes; Ctrl/⌘+Enter commits the focused field
  // (by blurring it) and closes. Esc inside a field is handled by fieldKeys.
  function onKeyDown(e: KeyboardEvent<HTMLDivElement>) {
    const action = editorKeyAction({
      key: e.key,
      kind: kindOf(e.target),
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
    });
    if (action === 'close' || action === 'submit') {
      e.preventDefault();
      e.stopPropagation(); // handled here: no page-level handler may act on it too
      if (e.target instanceof HTMLElement) e.target.blur();
      onClose();
    }
  }

  return (
    <div id={id} role="group" aria-label={`Rediger ${sample.code}`} className={styles.editor} onKeyDown={onKeyDown}>
      <div className={styles.fields}>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={titleId}>
            Titel
          </label>
          <input ref={titleInput} id={titleId} className={styles.input} {...title.props} onKeyDown={fieldKeys(title)} />
        </div>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={whatId}>
            Hvad er udtaget, og hvor
          </label>
          <input id={whatId} className={styles.input} {...what.props} onKeyDown={fieldKeys(what)} />
        </div>
      </div>
      <LinkPicker types={types} selected={linkedIds} onToggle={onToggleLink} />
      <div className={styles.editorActions}>
        <button
          type="button"
          className={styles.btnDanger}
          disabled={busy}
          onClick={() => {
            // The draft links the editor shows, not the server's.
            const text = deleteConfirmText({
              code: sample.code,
              title: sample.title,
              type_ids: linkedIds,
            });
            if (window.confirm(text)) onDelete();
          }}
        >
          Slet prøve
        </button>
        <button type="button" className={styles.btnGhost} onClick={onClose}>
          Luk
        </button>
      </div>
    </div>
  );
}

/**
 * One environmental sample (prototype `miljoe.png`): title + stage/result
 * pill + linked types, what was sampled, the stage chain, and the action the
 * stage allows. `Rediger` opens an inline editor for title, what, links and
 * delete.
 *
 * Each card owns its editor's text drafts (`useTextDraft`), so the parent
 * must render cards with `key={sample.id}` — then a draft can never move to
 * another sample when the list changes.
 */
export function SampleCard(props: SampleCardProps) {
  const { sample, types, linkedIds, busy, focused, editing, onEditing, onAdvance, onResult, onUndoResult, cardRef } =
    props;
  const editButton = useRef<HTMLButtonElement>(null);
  const pill = statusPill(sample);
  const action = cardAction(sample);
  const next = nextStage(sample.stage);
  const headingId = `sample-${sample.id}-title`;
  const editorId = `sample-${sample.id}-editor`;

  function closeEditor() {
    onEditing(false);
    editButton.current?.focus();
  }

  return (
    <article
      ref={cardRef}
      tabIndex={-1}
      className={focused ? `${styles.card} ${styles.focused}` : styles.card}
      aria-labelledby={headingId}
    >
      <header className={styles.head}>
        <h3 id={headingId} className={styles.title}>
          {sample.code} · {sample.title}
        </h3>
        <Pill tone={pill.tone}>{pill.label}</Pill>
        <p className={styles.linked}>
          <LinkedLine types={linkedTypes(linkedIds, types)} />
          <button
            ref={editButton}
            type="button"
            className={styles.textBtn}
            aria-expanded={editing}
            aria-controls={editing ? editorId : undefined}
            aria-describedby={headingId}
            onClick={() => onEditing(!editing)}
          >
            {editing ? 'Luk redigering' : 'Rediger'}
          </button>
        </p>
      </header>

      {sample.what && <p className={styles.what}>{sample.what}</p>}

      <StageChain sample={sample} />

      {action === 'advance' && (
        <div className={styles.actions}>
          <button
            type="button"
            className={styles.btnGhost}
            disabled={busy}
            onClick={onAdvance}
            aria-describedby={headingId}
            title={next ? `Markér som ${STAGE_LABEL[next]}` : undefined}
          >
            Næste trin <span aria-hidden="true">→</span>
          </button>
        </div>
      )}
      {action === 'answer' && (
        <div className={styles.actions}>
          <button type="button" className={styles.btnPrimary} disabled={busy} onClick={() => onResult('ren')}>
            Registrér svar: Ren
          </button>
          <button type="button" className={styles.btnGhost} disabled={busy} onClick={() => onResult('forurenet')}>
            Registrér svar: Forurenet
          </button>
        </div>
      )}
      {action === 'answered' && (
        <p className={styles.note}>
          {answeredNote({ type_ids: linkedIds })}{' '}
          <button
            type="button"
            className={styles.textBtn}
            disabled={busy}
            onClick={onUndoResult}
            aria-describedby={headingId}
          >
            Fortryd svar
          </button>
        </p>
      )}

      {editing && <SampleEditor {...props} id={editorId} onClose={closeEditor} />}
    </article>
  );
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Fragment, useRef, type KeyboardEvent, type RefObject } from 'react';

import type { ProjectInfo } from '../../api/types';
import { editorKeyAction } from '../../app/editorKeys';
import { kindOf } from '../../app/keyTargets';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import {
  META_FIELDS,
  metaPatch,
  parseYear,
  yearText,
  type MetaFieldSpec,
  type ProjectPatch,
} from '../../overblik/model';
import styles from './ProjectMetaForm.module.css';

export interface ProjectMetaFormProps {
  project: ProjectInfo | undefined;
  /** One field's sparse patch. Never gated on busy: the page queues it. */
  onCommit: (patch: ProjectPatch) => void;
  onInvalidYear: () => void;
  /** The required name was emptied and snapped back (`EMPTY_NAME_TOAST`). */
  onInvalidName: () => void;
  onClose: () => void;
}

/**
 * The case details, edited in place (R5). Each field commits on blur
 * (`useTextDraft`); an untouched blur sends nothing. Esc in a field drops its
 * draft without sending and parks focus on the form itself, so a second Esc
 * closes it (R10). Enter in a single-line field commits by leaving the field;
 * Ctrl/⌘+Enter commits the focused field and closes.
 */
export function ProjectMetaForm({ project, onCommit, onInvalidYear, onInvalidName, onClose }: ProjectMetaFormProps) {
  const formRef = useRef<HTMLElement>(null);

  // Esc outside a text field closes; Ctrl/⌘+Enter commits the focused field
  // (by blurring it) and closes. Esc inside a field is handled by fieldKeys.
  function onKeyDown(e: KeyboardEvent<HTMLElement>) {
    const action = editorKeyAction({
      key: e.key,
      kind: kindOf(e.target),
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
      altKey: e.altKey,
    });
    if (action === 'close' || action === 'submit') {
      e.preventDefault();
      e.stopPropagation(); // handled here: no page-level handler may act on it too
      if (e.target instanceof HTMLElement) e.target.blur();
      onClose();
    }
  }

  return (
    // tabIndex -1: a focus target for Esc-revert, never a Tab stop.
    <section
      ref={formRef}
      id="case-meta-form"
      className={styles.form}
      aria-label="Sagsoplysninger"
      tabIndex={-1}
      onKeyDown={onKeyDown}
    >
      <div className={styles.grid}>
        {META_FIELDS.map((spec) => (
          <Fragment key={spec.key}>
            <MetaTextField
              spec={spec}
              current={project?.[spec.key] ?? ''}
              onCommit={onCommit}
              onInvalid={onInvalidName}
              home={formRef}
            />
            {spec.key === 'building_address' && (
              <MetaYearField
                current={project?.year_of_construction}
                onCommit={onCommit}
                onInvalid={onInvalidYear}
                home={formRef}
              />
            )}
          </Fragment>
        ))}
      </div>
      <p className={styles.hint}>Ændringer gemmes, når du forlader feltet. Esc fortryder feltet; Esc igen lukker. Ctrl+Enter gemmer og lukker.</p>
    </section>
  );
}

function MetaTextField({
  spec,
  current,
  onCommit,
  onInvalid,
  home,
}: {
  spec: MetaFieldSpec;
  current: string;
  onCommit: (patch: ProjectPatch) => void;
  /** A required field was emptied; only the name is required. */
  onInvalid: () => void;
  home: RefObject<HTMLElement | null>;
}) {
  const draft = useTextDraft(current, (value) => onCommit(metaPatch(spec.key, value)), {
    required: spec.required,
    onInvalid,
  });
  const id = `meta-${spec.key}`;
  const keys = fieldKeys(draft, home, spec.multiline);
  return (
    <div className={spec.multiline ? `${styles.field} ${styles.wide}` : styles.field}>
      <label className={styles.label} htmlFor={id}>
        {spec.label}
      </label>
      {spec.multiline ? (
        <textarea id={id} className={styles.textarea} rows={3} {...draft.props} onKeyDown={keys} />
      ) : (
        <input id={id} type="text" className={styles.input} {...draft.props} onKeyDown={keys} />
      )}
    </div>
  );
}

function MetaYearField({
  current,
  onCommit,
  onInvalid,
  home,
}: {
  current: number | undefined;
  onCommit: (patch: ProjectPatch) => void;
  onInvalid: () => void;
  home: RefObject<HTMLElement | null>;
}) {
  const draft = useTextDraft<number | null>(
    yearText(current),
    (value) => onCommit({ year_of_construction: value }),
    { validate: (d) => parseYear(d), onInvalid },
  );
  return (
    <div className={styles.field}>
      <label className={styles.label} htmlFor="meta-year">
        Opført (år)
      </label>
      <input
        id="meta-year"
        type="text"
        inputMode="numeric"
        className={`${styles.input} mono`}
        {...draft.props}
        onKeyDown={fieldKeys(draft, home)}
      />
    </div>
  );
}

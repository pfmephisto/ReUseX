// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A small modal form over Kortlægning (Tilføj ressource, Tilføj kolonne).
 * Keys follow `formKeyDown`: Esc anywhere cancels (nothing is saved yet),
 * Ctrl/⌘+Enter submits; plain Enter submits natively. Tab is trapped inside
 * the dialog (`trapTab`, as in EditDialog). The page returns focus to the
 * table when the dialog closes.
 *
 * Its stylesheet also carries the field and button classes a form inside it
 * uses (`field`, `fieldLabel`, `input`, `btnGhost`, `btnPrimary`), composed
 * from the shared controls — a form imports those from here.
 */

import { useId, useRef, type ReactNode } from 'react';

import { formKeyDown } from '../../app/editorKeys';
import { trapTab } from '../../app/focusTrap';
import styles from './FormDialog.module.css';

export interface FormDialogProps {
  title: string;
  onCancel: () => void;
  onSubmit: () => void;
  children: ReactNode;
  /** The footer buttons; the primary one is `type="submit"`. */
  actions: ReactNode;
  error?: string | null;
}

export function FormDialog({ title, onCancel, onSubmit, children, actions, error }: FormDialogProps) {
  const titleId = useId();
  const formRef = useRef<HTMLFormElement>(null);
  return (
    <div className={styles.scrim} onMouseDown={(e) => e.target === e.currentTarget && onCancel()}>
      <form
        ref={formRef}
        className={styles.dialog}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        onSubmit={(e) => {
          e.preventDefault();
          onSubmit();
        }}
        onKeyDown={(e) => {
          trapTab(e, formRef.current);
          formKeyDown(e, { onCancel, onSubmit });
        }}
      >
        <h2 id={titleId} className={styles.title}>
          {title}
        </h2>
        <div className={styles.body}>{children}</div>
        {error && (
          <p className={styles.error} role="alert">
            {error}
          </p>
        )}
        <div className={styles.actions}>{actions}</div>
      </form>
    </div>
  );
}

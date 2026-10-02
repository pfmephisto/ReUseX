// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useRef, useState, type RefObject } from 'react';

import type { PropertyDefinition } from '../../api/types';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import { columnDeleteConfirm, columnKindLabel, hasOptions, optionsText } from '../../skabeloner/columns';
import styles from './ColumnList.module.css';

export interface ColumnListProps {
  columns: PropertyDefinition[];
  busy: boolean;
  /** An inline error per column id (a refused rename or options edit). */
  errors: Readonly<Record<string, string>>;
  /**
   * Bumped per field (`<id>:name`, `<id>:options`) when its commit was
   * refused, so that field drops its draft and shows the server value (R4-D1).
   */
  epochs: Readonly<Record<string, number>>;
  onRename: (column: PropertyDefinition, name: string) => void;
  onOptions: (column: PropertyDefinition, text: string) => void;
  onDelete: (column: PropertyDefinition) => void;
}

interface DraftFieldProps {
  value: string;
  label: string;
  home: RefObject<HTMLElement | null>;
  onCommit: (value: string) => void;
}

function NameField({ value, label, home, onCommit }: DraftFieldProps) {
  const draft = useTextDraft(value, onCommit, { required: true });
  return <input className={styles.input} aria-label={label} {...draft.props} onKeyDown={fieldKeys(draft, home)} />;
}

function OptionsField({ value, label, home, onCommit }: DraftFieldProps) {
  const draft = useTextDraft(value, onCommit);
  return (
    <label className={styles.field}>
      <span className={styles.fieldLabel}>{label}</span>
      <textarea
        className={styles.textarea}
        rows={Math.min(Math.max(value.split('\n').length, 2), 6)}
        {...draft.props}
        onKeyDown={fieldKeys(draft, home, true)}
      />
      <span className={styles.hint}>Én valgmulighed pr. linje, eller adskilt med komma.</span>
    </label>
  );
}

/**
 * "Egne felter" (R4-EF): the project's user columns — rename, edit a choice
 * column's options, delete. Fields commit on blur through the page; "Slet" is
 * a two-click confirm like TemplateList's, armed per column, disarmed on
 * blur or Esc.
 */
export function ColumnList({ columns, busy, errors, epochs, onRename, onOptions, onDelete }: ColumnListProps) {
  const home = useRef<HTMLElement | null>(null);
  const [armed, setArmed] = useState<string | null>(null);

  return (
    <section ref={home} tabIndex={-1} className={styles.panel} aria-labelledby="egne-felter-heading">
      <h3 id="egne-felter-heading" className={styles.heading}>
        Egne felter
      </h3>
      {columns.length === 0 ? (
        <p className={styles.empty}>Ingen egne felter endnu — tilføj dem fra Kortlægning.</p>
      ) : (
        <ul className={styles.list}>
          {columns.map((c) => {
            const isArmed = armed === c.id;
            const error = errors[c.id];
            return (
              <li key={c.id} className={styles.row}>
                <span className={styles.kind}>{columnKindLabel(c.type)}</span>
                <div className={styles.line}>
                  <NameField
                    key={epochs[`${c.id}:name`] ?? 0}
                    value={c.name}
                    label={`Navn på feltet ${c.name}`}
                    home={home}
                    onCommit={(name) => onRename(c, name)}
                  />
                  <button
                    type="button"
                    className={styles.btnDanger}
                    disabled={busy}
                    aria-label={isArmed ? `Bekræft: slet ${c.name}` : `Slet ${c.name}`}
                    onClick={() => {
                      if (!isArmed) {
                        setArmed(c.id);
                        return;
                      }
                      setArmed(null);
                      onDelete(c);
                    }}
                    onBlur={() => setArmed((a) => (a === c.id ? null : a))}
                    onKeyDown={(e) => {
                      if (e.key === 'Escape' && isArmed) {
                        e.preventDefault();
                        e.stopPropagation();
                        setArmed(null);
                      }
                    }}
                  >
                    {isArmed ? 'Bekræft: slet' : 'Slet'}
                  </button>
                </div>
                {hasOptions(c.type) && (
                  <OptionsField
                    key={epochs[`${c.id}:options`] ?? 0}
                    value={optionsText(c.options)}
                    label="Valgmuligheder"
                    home={home}
                    onCommit={(text) => onOptions(c, text)}
                  />
                )}
                {isArmed && (
                  <p className={styles.confirm} role="status">
                    {columnDeleteConfirm(c.name)}
                  </p>
                )}
                {error && (
                  <p className={styles.error} role="alert">
                    {error}
                  </p>
                )}
              </li>
            );
          })}
        </ul>
      )}
    </section>
  );
}

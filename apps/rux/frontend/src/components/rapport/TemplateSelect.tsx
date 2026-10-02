// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Template } from '../../api/types';
import { NO_TEMPLATE, parseTemplateChoice } from '../../rapport/model';
import styles from './TemplateSelect.module.css';

export interface TemplateSelectProps {
  id: string;
  label: string;
  templates: Template[];
  value: number | null;
  onChange: (id: number | null) => void;
  /** Offer "Ingen" as the first option (the Ressourcetabel picker). */
  allowNone?: boolean;
  disabled?: boolean;
}

/** A labelled native select over the project's templates (R9). */
export function TemplateSelect({ id, label, templates, value, onChange, allowNone, disabled }: TemplateSelectProps) {
  return (
    <label className={styles.field} htmlFor={id}>
      <span className={styles.fieldLabel}>{label}</span>
      <select
        id={id}
        className={styles.input}
        value={value === null ? NO_TEMPLATE : String(value)}
        disabled={disabled}
        onChange={(e) => onChange(parseTemplateChoice(e.target.value))}
      >
        {allowNone && <option value={NO_TEMPLATE}>Ingen</option>}
        {!allowNone && value === null && <option value={NO_TEMPLATE}>Vælg skabelon…</option>}
        {templates.map((t) => (
          <option key={t.id} value={String(t.id)}>
            {t.name}
          </option>
        ))}
      </select>
    </label>
  );
}

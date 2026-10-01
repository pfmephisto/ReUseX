// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { SurveyType } from '../../api/types';
import { ENV_LABEL, ENV_TONE } from '../../kortlaegning/vocab';
import { linkableTypes } from '../../miljoe/model';
import { Pill } from '../Pill';
import styles from './LinkPicker.module.css';

export interface LinkPickerProps {
  types: SurveyType[];
  /** The linked type ids as the user last set them. */
  selected: readonly number[];
  onToggle: (typeId: number) => void;
  legend?: string;
}

/**
 * The survey types a sample can cover, each with its current miljøstatus, as
 * checkboxes. Each toggle is a field commit — the caller sends it at once.
 * Rejected types are hidden unless already linked; then they show "Afvist"
 * so they can be unlinked.
 */
export function LinkPicker({ types, selected, onToggle, legend = 'Koblet til typer' }: LinkPickerProps) {
  const options = linkableTypes(types, selected);
  return (
    <fieldset className={styles.picker}>
      <legend className={styles.legend}>{legend}</legend>
      {options.length === 0 ? (
        <p className={styles.empty}>Ingen typer i kortlægningen endnu — prøven kan kobles senere.</p>
      ) : (
        <ul className={styles.list}>
          {options.map((t) => (
            <li key={t.id}>
              <label className={styles.option}>
                <input
                  type="checkbox"
                  className={styles.checkbox}
                  checked={selected.includes(t.id)}
                  onChange={() => onToggle(t.id)}
                />
                <span className={styles.name}>{t.name}</span>
                {t.review_status === 'rejected' ? (
                  <Pill tone="wait">Afvist</Pill>
                ) : (
                  <Pill tone={ENV_TONE[t.environment_status]}>{ENV_LABEL[t.environment_status]}</Pill>
                )}
              </label>
            </li>
          ))}
        </ul>
      )}
    </fieldset>
  );
}

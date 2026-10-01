// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Ref } from 'react';

import styles from './CaseHero.module.css';

export interface CaseHeroProps {
  name: string;
  /** Joined metadata, or '' when the record has none yet. */
  subline: string;
  editing: boolean;
  onToggle: () => void;
  /** Focus returns here when the editor closes. */
  toggleRef: Ref<HTMLButtonElement>;
}

/** The case's navy hero (prototype `.hero`): name, metadata line, edit toggle. */
export function CaseHero({ name, subline, editing, onToggle, toggleRef }: CaseHeroProps) {
  return (
    <section className={styles.hero} aria-labelledby="case-name">
      <h2 id="case-name" className={styles.name}>
        {name}
      </h2>
      <p className={subline ? styles.sub : `${styles.sub} ${styles.empty}`}>
        {subline || 'Ingen sagsoplysninger endnu — adresse, byggeår og registrering tilføjes her.'}
      </p>
      <button
        ref={toggleRef}
        type="button"
        className={styles.edit}
        aria-expanded={editing}
        aria-controls={editing ? 'case-meta-form' : undefined}
        onClick={onToggle}
      >
        {editing ? 'Luk redigering' : 'Rediger sagsoplysninger'}
      </button>
    </section>
  );
}

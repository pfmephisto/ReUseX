// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import styles from './TypeMark.module.css';

/** What a type-scoped key's marker says: a write changes every part of the type. */
export const TYPE_MARK_TEXT = 'Gælder alle dele af typen';

/**
 * The "type" marker on a type-scoped key — in the table header and in Alle
 * egenskaber alike. Sighted users read "type" (with the full sentence as a
 * tooltip); a screen reader hears the sentence instead.
 */
export function TypeMark() {
  return (
    <span className={styles.typeMark} title={TYPE_MARK_TEXT}>
      <span aria-hidden="true">type</span>
      <span className={styles.srOnly}>({TYPE_MARK_TEXT.toLowerCase()})</span>
    </span>
  );
}

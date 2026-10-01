// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import styles from './Toast.module.css';

export interface ToastProps {
  /** The current message, or null when nothing is showing. Pair with `useToast`. */
  message: string | null;
}

/** A transient bottom-centre confirmation, e.g. "Kopieret", "Gemt". */
export function Toast({ message }: ToastProps) {
  return (
    <div className={`${styles.toast} ${message ? styles.show : ''}`} role="status" aria-live="polite">
      {message}
    </div>
  );
}

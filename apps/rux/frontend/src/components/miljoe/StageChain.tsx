// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Sample } from '../../api/types';
import { chainSteps, type StepState } from '../../miljoe/model';
import styles from './StageChain.module.css';

/**
 * What a screen reader hears after each step's label, so the chain reads as
 * text. The current step is announced by `aria-current` instead.
 */
const STATE_TEXT: Record<StepState, string | null> = {
  done: 'udført',
  current: null,
  todo: 'ikke nået',
};

/**
 * Planlagt — Udtaget — Sendt til lab — Svar modtaget, with the sample's stage
 * current. Done steps have a filled dot, the current one a filled, ringed dot
 * and todo steps a hollow one, so the state never rests on colour alone.
 */
export function StageChain({ sample }: { sample: Pick<Sample, 'stage'> }) {
  return (
    <ol className={styles.chain} aria-label="Prøveforløb">
      {chainSteps(sample).map((step) => {
        const stateText = STATE_TEXT[step.state];
        return (
          <li
            key={step.stage}
            className={`${styles.step} ${styles[step.state]}`}
            aria-current={step.state === 'current' ? 'step' : undefined}
          >
            <span className={styles.dot} aria-hidden="true" />
            {step.label}
            {stateText && <span className={styles.srOnly}> ({stateText})</span>}
          </li>
        );
      })}
    </ol>
  );
}

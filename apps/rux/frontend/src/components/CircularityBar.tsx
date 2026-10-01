// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { circToken, formatTonnes } from '../kortlaegning/vocab';
import { CIRC_EMPTY_TEXT, circularityAriaLabel, type CircSegment } from '../overblik/model';
import styles from './CircularityBar.module.css';

export interface CircularityBarProps {
  segments: CircSegment[];
  /** Overblik shows the legend; Rapport's hero shows the bar alone. */
  legend?: boolean;
}

/**
 * The affaldshierarki as one stacked bar: each step's share of the tonnes in
 * its --circ-* colour (via `circToken`). Widths are flex-grow by tonnes, so no
 * percentage is a style literal; the legend prints the whole percents (largest
 * remainder, always summing to 100). The bar carries the split in words as its
 * accessible name, so colour is never the only carrier; with no tonnes it
 * draws an empty track and says so.
 */
export function CircularityBar({ segments, legend = true }: CircularityBarProps) {
  const empty = segments.length === 0;
  return (
    <div className={styles.wrap}>
      <div className={styles.bar} role="img" aria-label={circularityAriaLabel(segments)}>
        {segments.map((s) => (
          <span
            key={s.treatment}
            className={styles.segment}
            style={{ flexGrow: s.tonnes, background: circToken(s.treatment) }}
            title={`${s.label}: ${formatTonnes(s.tonnes)} (${s.percent} %)`}
          />
        ))}
      </div>
      {legend &&
        (empty ? (
          <p className={styles.empty}>{CIRC_EMPTY_TEXT}</p>
        ) : (
          <ul className={styles.legend} aria-label="Signaturforklaring">
            {segments.map((s) => (
              <li key={s.treatment} className={styles.item}>
                <i className={styles.swatch} style={{ background: circToken(s.treatment) }} aria-hidden="true" />
                {s.label} <b className="mono">{s.percent} %</b>
              </li>
            ))}
          </ul>
        ))}
    </div>
  );
}

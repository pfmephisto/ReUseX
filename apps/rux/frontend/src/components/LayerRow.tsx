// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { CloudInfo } from '../api/types';
import type { CloudStreamState } from '../viewport/useCloudStream';
import styles from './LayerRow.module.css';

export interface LayerRowProps {
  cloud: CloudInfo;
  visible: boolean;
  /** Undefined until the layer has been switched on at least once. */
  progress?: CloudStreamState;
  onToggle: (visible: boolean) => void;
}

const NUMBER = new Intl.NumberFormat();

/**
 * One row of the layer panel.
 *
 * Shows the download state as well as visibility, because with a JSON transport
 * a large cloud takes long enough that "checked but nothing on screen yet" is a
 * state the user will hit and must be able to tell apart from "checked and
 * broken". Once #283 lands and pages arrive in milliseconds this readout stops
 * mattering — but it will still be correct.
 */
export function LayerRow({ cloud, visible, progress, onToggle }: LayerRowProps) {
  const fraction = progress?.fraction ?? null;
  const streaming = progress !== undefined && !progress.done && !progress.error;

  return (
    <div className={styles.row}>
      <label className={styles.main}>
        <input
          type="checkbox"
          checked={visible}
          onChange={(event) => onToggle(event.target.checked)}
          className={styles.checkbox}
        />
        <span className={styles.name} title={cloud.name}>
          {cloud.name}
        </span>
      </label>

      <div className={styles.meta}>
        <span className={styles.type}>{cloud.type}</span>
        <span className={`${styles.count} mono`}>{NUMBER.format(cloud.point_count)}</span>
      </div>

      {streaming && (
        <div className={styles.track}>
          <div
            className={`${styles.fill} ${fraction === null ? styles.indeterminate : ''}`}
            style={fraction === null ? undefined : { width: `${fraction * 100}%` }}
          />
        </div>
      )}

      {progress?.error && (
        <p className={styles.error} title={progress.error.message}>
          {progress.error.message}
        </p>
      )}
    </div>
  );
}

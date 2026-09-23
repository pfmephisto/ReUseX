// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';
import { useSearchParams } from 'react-router-dom';

import { LabelsPane } from '../components/LabelsPane';
import styles from './DataPage.module.css';

/**
 * Label legend editor at its own top-level route (`/labels`).
 *
 * Formerly one of three tabs in the old DataPage tab model. After Materials and
 * Components were promoted to their own routes (issue #451) Labels was the
 * remaining pane, so it became a standalone page rather than keeping the tab
 * chrome for a single item.
 *
 * The `?cloud=<name>` URL param is preserved — links from the old
 * `?tab=labels&cloud=<name>` still carry the relevant param here.
 */
export function LabelsPage() {
  const [params, setParams] = useSearchParams();

  const setParam = useCallback(
    (updates: Record<string, string | null>) => {
      const next = new URLSearchParams(params);
      for (const [key, value] of Object.entries(updates)) {
        if (value === null) next.delete(key);
        else next.set(key, value);
      }
      setParams(next, { replace: true });
    },
    [params, setParams],
  );

  return (
    <div className={styles.page}>
      <LabelsPane
        cloud={params.get('cloud')}
        onCloudChange={(cloud) => setParam({ cloud })}
      />
    </div>
  );
}

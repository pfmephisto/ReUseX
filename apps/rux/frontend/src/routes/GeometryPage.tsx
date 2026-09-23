// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';
import { useSearchParams } from 'react-router-dom';

import { ComponentsPane } from '../components/ComponentsPane';
import styles from './GeometryPage.module.css';

/**
 * Building-component inventory at its own top-level route.
 *
 * Moved here from the old DataPage tab model (issue #451). The URL params
 * `?component=<name>` and `?type=<type>` are preserved, so links from earlier
 * DataPage sessions remain valid if the user replaces `/data?tab=components`
 * with `/geometry`.
 */
export function GeometryPage() {
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
      <ComponentsPane
        selected={params.get('component')}
        onSelect={(name) => setParam({ component: name })}
        type={params.get('type')}
        onTypeChange={(type) => setParam({ type, component: null })}
      />
    </div>
  );
}

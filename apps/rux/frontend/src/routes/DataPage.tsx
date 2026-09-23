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
 * The page shows two separately-managed sections (issue #452):
 *
 * - **Geometry labels** — structural segmentation clouds (`planes`, `rooms`).
 *   Names here describe spatial structure; renaming is purely cosmetic.
 * - **Semantic labels** — object-class clouds (`instances` and any annotation-
 *   derived cloud). Names carry ML class identity; `instances` is read-only
 *   because its format is parsed back by the pipeline.
 *
 * Each section carries its own cloud-selection URL param so links remain stable:
 * `?geo=<name>` for geometry and `?sem=<name>` for semantic.  The old bare
 * `?cloud=<name>` param is no longer used; it silently falls out of the URL
 * on first navigation without causing an error.
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
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Geometry Labels</h2>
        <p className={styles.sectionDesc}>
          Structural segmentation clouds: planes and rooms. Names are cosmetic labels only.
        </p>
        <LabelsPane
          kind="geometry"
          cloud={params.get('geo')}
          onCloudChange={(cloud) => setParam({ geo: cloud })}
        />
      </section>

      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Semantic Labels</h2>
        <p className={styles.sectionDesc}>
          Object-class clouds produced by ML annotation and instance segmentation.
        </p>
        <LabelsPane
          kind="semantic"
          cloud={params.get('sem')}
          onCloudChange={(cloud) => setParam({ sem: cloud })}
        />
      </section>
    </div>
  );
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';
import { useSearchParams } from 'react-router-dom';

import { ComponentsPane } from '../components/ComponentsPane';
import { LabelsPane } from '../components/LabelsPane';
import { MaterialsPane } from '../components/MaterialsPane';
import { DATA_TABS, DATA_TAB_LABELS, parseDataTab, type DataTab } from '../data/tabs';
import styles from './DataPage.module.css';

/**
 * The project's non-geometric data: components, passports, label legends.
 *
 * Three panes under one route rather than three routes, because they are one
 * screen conceptually — what this project *knows*, as opposed to what it looks
 * like — and because two of them are editors that share the write model
 * (optimistic, sparse patch, 409 vs 503). Splitting them would have duplicated
 * that model three ways.
 *
 * Every pane's selection lives in the URL alongside the tab, so a link carries
 * the whole screen: `?tab=materials&material=<guid>` opens that passport's
 * editor. Selections are namespaced per pane rather than sharing one `?id=`,
 * so switching tabs does not carry a component name into the material pane and
 * ask the server for a passport that cannot exist.
 */
export function DataPage() {
  const [params, setParams] = useSearchParams();
  const tab = parseDataTab(params.get('tab'));

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

  const selectTab = useCallback(
    (next: DataTab) => setParam({ tab: next === 'components' ? null : next }),
    [setParam],
  );

  return (
    <div className={styles.page}>
      <div className={styles.tabs} role="tablist" aria-label="Project data">
        {DATA_TABS.map((entry) => (
          <button
            key={entry}
            type="button"
            role="tab"
            aria-selected={entry === tab}
            className={`${styles.tab} ${entry === tab ? styles.active : ''}`}
            onClick={() => selectTab(entry)}
          >
            {DATA_TAB_LABELS[entry]}
          </button>
        ))}
      </div>

      <div className={styles.body} role="tabpanel" aria-label={DATA_TAB_LABELS[tab]}>
        {tab === 'components' && (
          <ComponentsPane
            selected={params.get('component')}
            onSelect={(name) => setParam({ component: name })}
            type={params.get('type')}
            // Changing the filter drops the selection: the selected component
            // may not be in the new list, and a detail pane showing a row the
            // table does not contain is a screen disagreeing with itself.
            onTypeChange={(type) => setParam({ type, component: null })}
          />
        )}
        {tab === 'materials' && (
          <MaterialsPane
            selected={params.get('material')}
            onSelect={(guid) => setParam({ material: guid })}
          />
        )}
        {tab === 'labels' && (
          <LabelsPane
            cloud={params.get('cloud')}
            onCloudChange={(cloud) => setParam({ cloud })}
          />
        )}
      </div>
    </div>
  );
}

// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The `/data` tab model.
 *
 * The tab lives in `?tab=` rather than in component state so that a pane is a
 * link — the same reason the viewport carries `?cloud=` and the frame browser
 * carries `?frame=`. Three panes that can only be reached by clicking would
 * make "the materials editor" unshareable and unbookmarkable, and would lose
 * the pane on every reload.
 */

export type DataTab = 'components' | 'materials' | 'labels';

export const DATA_TABS: readonly DataTab[] = ['components', 'materials', 'labels'];

export const DATA_TAB_LABELS: Record<DataTab, string> = {
  components: 'Components',
  materials: 'Materials',
  labels: 'Labels',
};

/**
 * Read `?tab=`, defaulting to components.
 *
 * An unrecognised value lands on the default rather than on an error screen:
 * a mistyped tab name in a shared link should show the user the page, not a
 * fault. Components is the default because it is the read-only pane — arriving
 * at an editor you did not ask for is the wrong first frame.
 */
export function parseDataTab(raw: string | null | undefined): DataTab {
  return DATA_TABS.includes(raw as DataTab) ? (raw as DataTab) : 'components';
}

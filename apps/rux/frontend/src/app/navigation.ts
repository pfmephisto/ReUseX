// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The navigation model for the whole app, as data.
 *
 * Two groups: the case workflow a surveyor works through (`sag`, in the
 * prototype-v2 order) and the technical tools that existed before it
 * (`tools`). Not-yet-built destinations are listed with a `pending` reason and
 * rendered inert rather than hidden — a user who cannot see that a place will
 * exist reasonably concludes the GUI cannot do that thing at all.
 *
 * Kept free of React so the contract is unit-testable in Node.
 */

import type { Health, ProjectSummary } from '../api/types';

export type NavGroup = 'sag' | 'tools';

/** Live counts the sidebar can show next to an entry. */
export type NavBadge = 'reviewQueue' | 'pendingSamples';

export interface NavEntry {
  to: string;
  label: string;
  group: NavGroup;
  /** Match exactly, for a route with a longer route nested under it. */
  end?: boolean;
  /** Set while the destination does not exist yet; says when it arrives. */
  pending?: string;
  badge?: NavBadge;
}

export const ALL_CASES_PATH = '/sager';

export const NAV_ENTRIES: readonly NavEntry[] = [
  { to: '/', label: 'Overblik', group: 'sag', end: true },
  {
    to: '/kortlaegning',
    label: 'Kortlægning',
    group: 'sag',
    badge: 'reviewQueue',
    pending: 'Kommer i fase 3 — brug Materialedata indtil da',
  },
  {
    to: '/miljoe',
    label: 'Miljø & prøver',
    group: 'sag',
    badge: 'pendingSamples',
    pending: 'Kommer i fase 4 — prøver og miljøstatus',
  },
  { to: '/rapport', label: 'Rapport', group: 'sag', pending: 'Kommer i fase 5 — rapportversioner' },
  {
    to: '/indberetning',
    label: 'Indberetning',
    group: 'sag',
    pending: 'Kommer i fase 5 — fraktioner til bygningsaffald.dk',
  },

  { to: '/viewport', label: 'Viewport', group: 'tools' },
  { to: '/graph-view', label: 'Posegraf', group: 'tools' },
  { to: '/pipeline', label: 'Pipeline', group: 'tools', end: true },
  { to: '/pipeline/log', label: 'Kørselslog', group: 'tools' },
  { to: '/frames', label: 'Billeder', group: 'tools' },
  { to: '/geometry', label: 'Geometri', group: 'tools' },
  { to: '/instances', label: 'Instanser', group: 'tools' },
  { to: '/materials', label: 'Materialedata', group: 'tools' },
  { to: '/labels', label: 'Labels', group: 'tools' },
  { to: '/export', label: 'Eksport', group: 'tools' },
];

export function entriesIn(group: NavGroup): NavEntry[] {
  return NAV_ENTRIES.filter((e) => e.group === group);
}

/** The badge label for a count, or null when there is nothing to flag. */
export function badgeText(count: number | undefined): string | null {
  if (count === undefined || count <= 0) return null;
  return count > 99 ? '99+' : String(count);
}

/**
 * The name the sidebar shows under PROJEKT: the building's record name when the
 * project metadata has one, else the `.rux` file name from `/health`.
 */
export function displayProjectName(
  summary: ProjectSummary | undefined,
  health: Health | undefined,
): string | undefined {
  const recordName = summary?.projects?.[0]?.name?.trim();
  return recordName ? recordName : health?.project.name;
}

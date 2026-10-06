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
import { editorKeyAction } from './editorKeys';
import type { TargetKind } from './keyTargets';
import {
  INDBERETNING_PATH,
  KORTLAEGNING_PATH,
  MILJOE_PATH,
  OVERBLIK_PATH,
  PROJEKTDATA_PATH,
  RAPPORT_PATH,
  SEGMENTERING_PATH,
  SKABELONER_PATH,
} from './links';

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

/**
 * The shell's phone breakpoint (Phase 6 R5): at or below it the sidebar is a
 * drawer behind the title bar's Menu button. 900px, in rem like the existing
 * `45rem` query in controls.module.css. CSS cannot import it, so the
 * `@media` rules in AppShell, Sidebar and TitleBar spell the same value.
 */
export const DRAWER_QUERY = '(max-width: 56.25rem)';

/**
 * What a key does to the open drawer. Esc closes it from anywhere in the shell
 * except a text field, where R10 gives Esc to the field (revert its draft).
 */
export function drawerKeyAction(k: { key: string; kind: TargetKind; open: boolean }): 'close' | null {
  if (!k.open) return null;
  return editorKeyAction({ key: k.key, kind: k.kind }) === 'close' ? 'close' : null;
}

/**
 * Whether a click inside the drawer closes it: any link does, the link to the
 * page already shown included, since the user has chosen where to be. `tags`
 * are the tag names from the click target up to the drawer. A pending entry
 * is a span, so it leaves the drawer open.
 */
export function drawerClickCloses(tags: readonly string[]): boolean {
  return tags.some((t) => t.toUpperCase() === 'A');
}

export const NAV_ENTRIES: readonly NavEntry[] = [
  { to: OVERBLIK_PATH, label: 'Overblik', group: 'sag', end: true },
  { to: KORTLAEGNING_PATH, label: 'Kortlægning', group: 'sag', badge: 'reviewQueue' },
  { to: '/viewport', label: 'Viewport', group: 'sag' },
  { to: SEGMENTERING_PATH, label: 'Segmentering', group: 'sag' },
  { to: MILJOE_PATH, label: 'Miljø & prøver', group: 'sag', badge: 'pendingSamples' },
  { to: RAPPORT_PATH, label: 'Rapport', group: 'sag' },
  { to: INDBERETNING_PATH, label: 'Indberetning', group: 'sag' },
  { to: SKABELONER_PATH, label: 'Skabeloner', group: 'sag' },

  { to: PROJEKTDATA_PATH, label: 'Projektdata', group: 'tools' },
  { to: '/graph-view', label: 'Posegraf', group: 'tools' },
  { to: '/pipeline', label: 'Pipeline', group: 'tools', end: true },
  { to: '/pipeline/log', label: 'Kørselslog', group: 'tools' },
  { to: '/frames', label: 'Billeder', group: 'tools' },
  { to: '/geometry', label: 'Geometri', group: 'tools' },
  { to: '/instances', label: 'Instanser', group: 'tools' },
  { to: '/labels', label: 'Labels', group: 'tools' },
];

/** An old path that still resolves, so bookmarks and cross-links keep working. */
export interface Redirect {
  from: string;
  to: string;
}

/**
 * Retired paths (resources/templates spec §3), rendered by App.tsx as
 * `<Navigate replace>` ahead of the catch-all, so they never stack history.
 * The query is dropped. A source must not also be a `<Route>` in App.tsx:
 * whoever adds one here deletes that route in the same change. Phase 4 adds
 * `/export`.
 */
export const REDIRECTS: readonly Redirect[] = [
  { from: '/on-site', to: KORTLAEGNING_PATH },
  { from: '/onsite', to: KORTLAEGNING_PATH },
  { from: '/materials', to: KORTLAEGNING_PATH },
  { from: '/export', to: RAPPORT_PATH },
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

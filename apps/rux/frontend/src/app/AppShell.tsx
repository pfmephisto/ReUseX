// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import { useLocation } from 'react-router-dom';

import { api } from '../api/client';
import type { Health, ProjectSummary, SurveySummary } from '../api/types';
import { TitleBar } from '../components/TitleBar';
import { Sidebar } from '../components/Sidebar';
import { JobToaster } from '../components/JobToaster';
import { useAsync } from './useAsync';
import { useJobs } from './JobsContext';
import { SurveyCountsProvider } from './SurveyCountsContext';
import { kindOf } from './keyTargets';
import { DRAWER_QUERY, displayProjectName, drawerKeyAction } from './navigation';
import styles from './AppShell.module.css';

const NAV_ID = 'app-nav';

/**
 * Title bar + sidebar + content region.
 *
 * The shell resolves the project identity once, from `GET /health`, rather than
 * from `/project`: health is the cheap call, it is the one the contract
 * designates for the version handshake, and it reports `project.open === false`
 * when the database could not be opened — which is exactly the state a title
 * bar must not render as if everything were fine.
 *
 * The Kortlægning and Miljø & prøver badges both come from
 * `GET /survey/summary` (`counts.queue`, `pending_samples`: samples not yet at
 * *svar*, i.e. those that can hold a type at *afventer prøve*). A server that
 * predates the survey routes answers 404; the shell then shows no badge rather
 * than an error — the badge is a hint, not something to block the app on.
 *
 * Below 900px (Phase 6 R5) the sidebar is a drawer behind the title bar's
 * Menu button, over a scrim. Open, it takes focus and `<main>` is inert, so
 * Tab cannot wander behind the scrim; the title bar stays live, since Menu is
 * how it closes. It closes on any link click inside it, on a scrim click, on
 * Esc (except in a text field, R10) and on any navigation, and every close
 * hands focus back to Menu so it never drops to <body>. Widening past the
 * breakpoint closes it too, so a desktop is never left with an inert `<main>`.
 */
export function AppShell({ children }: { children: ReactNode }) {
  const { data: health, error } = useAsync<Health>((signal) => api.health(signal), []);
  const project = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);
  const summary = project.data;
  const survey = useAsync<SurveySummary>((signal) => api.surveySummary(signal), []);
  const { active, status } = useJobs();
  const reviewQueue = survey.error ? undefined : survey.data?.counts.queue;
  const pendingSamples = survey.error ? undefined : survey.data?.pending_samples;
  const surveyCounts = useMemo(
    () => ({ refresh: survey.reload, refreshProject: project.reload }),
    [survey.reload, project.reload],
  );

  const [navOpen, setNavOpen] = useState(false);
  const menuRef = useRef<HTMLButtonElement>(null);
  const navRef = useRef<HTMLElement>(null);
  const closeNav = useCallback(() => {
    setNavOpen(false);
    menuRef.current?.focus();
  }, []);

  const location = useLocation();
  useEffect(() => {
    setNavOpen(false); // any navigation closes the drawer
  }, [location.pathname, location.search]);

  useEffect(() => {
    if (!navOpen) return;
    navRef.current?.focus();
    const onKey = (e: KeyboardEvent) => {
      if (drawerKeyAction({ key: e.key, kind: kindOf(e.target), open: true }) !== 'close') return;
      e.preventDefault();
      e.stopPropagation();
      closeNav();
    };
    const wide = window.matchMedia(DRAWER_QUERY);
    const onWidth = () => {
      if (!wide.matches) setNavOpen(false);
    };
    document.addEventListener('keydown', onKey, true);
    wide.addEventListener('change', onWidth);
    return () => {
      document.removeEventListener('keydown', onKey, true);
      wide.removeEventListener('change', onWidth);
    };
  }, [navOpen, closeNav]);

  return (
    <div className={styles.shell}>
      <TitleBar
        projectName={health?.project.name}
        projectOpen={health?.project.open}
        schemaVersion={health?.project.schema_version}
        version={health?.version}
        implementation={health?.implementation}
        connection={status}
        activeJobCount={active.length}
        unreachable={Boolean(error)}
        menuOpen={navOpen}
        onMenu={() => (navOpen ? closeNav() : setNavOpen(true))}
        menuRef={menuRef}
        menuControls={NAV_ID}
      />
      <div className={styles.body}>
        <div className={styles.scrim} hidden={!navOpen} aria-hidden="true" onClick={closeNav} />
        <Sidebar
          id={NAV_ID}
          open={navOpen}
          onClose={closeNav}
          navRef={navRef}
          projectName={displayProjectName(summary, health)}
          badges={{ reviewQueue, pendingSamples }}
        />
        <main className={styles.content} inert={navOpen}>
          <SurveyCountsProvider value={surveyCounts}>{children}</SurveyCountsProvider>
        </main>
      </div>
      <JobToaster />
    </div>
  );
}

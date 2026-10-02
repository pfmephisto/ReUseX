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
 * Esc (except in a text field, R10) and on any other navigation (back/forward,
 * a programmatic redirect); each of those hands focus back to Menu. Crossing
 * the breakpoint closes it too, so a desktop is never left with an inert
 * `<main>`, and moves focus off whatever the crossing hid: widening, from
 * the drawer or Menu to the sidebar's active link (else `<main>`);
 * narrowing, from the sidebar to Menu.
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
  const navOpenRef = useRef(false);
  navOpenRef.current = navOpen;
  const menuRef = useRef<HTMLButtonElement>(null);
  const navRef = useRef<HTMLElement>(null);
  const mainRef = useRef<HTMLElement>(null);
  const closeNav = useCallback(() => {
    setNavOpen(false);
    menuRef.current?.focus();
  }, []);

  const location = useLocation();
  useEffect(() => {
    // Any navigation closes an open drawer. Never on first mount, and never
    // when it is already closed: a desktop route change must not pull focus.
    if (navOpenRef.current) closeNav();
  }, [location.pathname, location.search, closeNav]);

  // `<main>` is inert until the close re-renders, so a focus move to it waits.
  const [focusMain, setFocusMain] = useState(false);
  useEffect(() => {
    if (!focusMain || navOpen) return;
    setFocusMain(false);
    mainRef.current?.focus();
  }, [focusMain, navOpen]);

  useEffect(() => {
    const drawerMq = window.matchMedia(DRAWER_QUERY);
    // The browser may drop focus from an element the crossing hid before the
    // change event fires, so remember the last real focus target.
    let lastFocused: Element | null = null;
    const onFocusIn = (e: FocusEvent) => {
      lastFocused = e.target instanceof Element ? e.target : null;
    };
    const onCross = () => {
      const focused = document.activeElement === document.body ? lastFocused : document.activeElement;
      const inNav = navRef.current?.contains(focused) ?? false;
      setNavOpen(false);
      if (drawerMq.matches) {
        if (inNav) menuRef.current?.focus(); // the sidebar just went off-canvas
        return;
      }
      if (!inNav && focused !== menuRef.current) return; // Menu just went display:none
      const active = navRef.current?.querySelector<HTMLElement>('[aria-current="page"]');
      if (active) active.focus();
      else setFocusMain(true);
    };
    document.addEventListener('focusin', onFocusIn);
    drawerMq.addEventListener('change', onCross);
    return () => {
      document.removeEventListener('focusin', onFocusIn);
      drawerMq.removeEventListener('change', onCross);
    };
  }, []);

  useEffect(() => {
    if (!navOpen) return;
    navRef.current?.focus();
    const onKey = (e: KeyboardEvent) => {
      if (drawerKeyAction({ key: e.key, kind: kindOf(e.target), open: true }) !== 'close') return;
      e.preventDefault();
      e.stopPropagation();
      closeNav();
    };
    document.addEventListener('keydown', onKey, true);
    return () => document.removeEventListener('keydown', onKey, true);
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
        <main ref={mainRef} className={styles.content} inert={navOpen} tabIndex={-1}>
          <SurveyCountsProvider value={surveyCounts}>{children}</SurveyCountsProvider>
        </main>
      </div>
      <JobToaster />
    </div>
  );
}

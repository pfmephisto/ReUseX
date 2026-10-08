// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect } from 'react';

import { casesApi } from '../api/cases';
import { TitleBar } from '../components/TitleBar';
import { Spinner } from '../components/Spinner';
import { SagerPage } from '../routes/SagerPage';
import { useAuth } from './AuthGate';
import { CASES_PATH, legacyRedirectTarget, readLastCase } from './cases';
import { useTheme } from './useTheme';
import styles from './AppShell.module.css';

/**
 * The case list at `/sager`: the title bar over Sager, outside any case — no
 * sidebar, no events socket (those belong to a case).
 */
export function CasesApp() {
  const theme = useTheme();
  const auth = useAuth();
  return (
    <div className={styles.shell}>
      <TitleBar
        projectName="Sager"
        themePreference={theme.preference}
        onThemeChange={theme.setPreference}
        user={auth?.me.mode === 'server' ? auth.me.user : undefined}
        onLogout={auth ? () => void auth.logout() : undefined}
      />
      <div className={styles.body}>
        <main className={styles.content} tabIndex={-1}>
          <SagerPage />
        </main>
      </div>
    </div>
  );
}

/**
 * An old, unprefixed path (a bookmark from before cases): on to the same
 * place in the last-used case if the server still has it, else to the list.
 */
export function LegacyRedirect({ pathname, search }: { pathname: string; search: string }) {
  useEffect(() => {
    const controller = new AbortController();
    casesApi
      .list(controller.signal)
      .then(
        (list) =>
          legacyRedirectTarget(
            pathname,
            search,
            readLastCase(),
            list.cases.map((c) => c.id),
          ),
        () => CASES_PATH,
      )
      .then((target) => {
        if (!controller.signal.aborted) window.location.replace(target);
      });
    return () => controller.abort();
  }, [pathname, search]);
  return <Spinner label="Finder sagen…" />;
}

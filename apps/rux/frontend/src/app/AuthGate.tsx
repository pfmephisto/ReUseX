// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { createContext, useCallback, useContext, useEffect, useState, type ReactNode } from 'react';

import { authApi } from '../api/auth';
import { ApiRequestError } from '../api/client';
import type { AuthMe } from '../api/types';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { authBootAction, LOGIN_PATH } from './auth';
import { setLoginRedirect } from './unauthorized';
import styles from './AuthGate.module.css';

export interface AuthState {
  me: AuthMe;
  /** Ends the session and goes to the login page (server mode only). */
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthState | null>(null);

/** Who is signed in. Only inside an AuthGate. */
export function useAuth(): AuthState | null {
  return useContext(AuthContext);
}

/**
 * Asks the server who this browser is before anything else renders. Signed
 * out (401): on to the login page, which comes back here. In local mode the
 * server answers with its implicit user, so the login page never shows.
 */
export function AuthGate({ children }: { children: ReactNode }) {
  const [me, setMe] = useState<AuthMe | null>(null);
  const [error, setError] = useState<Error | null>(null);
  const [attempt, setAttempt] = useState(0);

  useEffect(() => {
    const controller = new AbortController();
    setError(null);
    authApi.me(controller.signal).then(
      (answer) => {
        if (controller.signal.aborted) return;
        setLoginRedirect(answer.mode === 'server');
        setMe(answer);
      },
      (cause: unknown) => {
        if (controller.signal.aborted) return;
        const action = authBootAction(
          { ok: false, status: cause instanceof ApiRequestError ? cause.status : undefined },
          window.location.pathname + window.location.search,
        );
        if (action.kind === 'login') window.location.replace(action.href);
        else setError(cause instanceof Error ? cause : new Error(String(cause)));
      },
    );
    return () => controller.abort();
  }, [attempt]);

  const logout = useCallback(async () => {
    try {
      await authApi.logout();
    } finally {
      window.location.assign(LOGIN_PATH);
    }
  }, []);

  if (error)
    return (
      <div className={styles.wait}>
        <ErrorBanner error={error} onRetry={() => setAttempt((n) => n + 1)} context="login-oplysningerne" />
      </div>
    );
  if (!me)
    return (
      <div className={styles.wait}>
        <Spinner label="Tjekker login…" />
      </div>
    );
  return <AuthContext.Provider value={{ me, logout }}>{children}</AuthContext.Provider>;
}

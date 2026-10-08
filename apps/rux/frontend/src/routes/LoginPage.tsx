// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState, type FormEvent } from 'react';

import { authApi, LoginError } from '../api/auth';
import { loginErrorText, safeNext } from '../app/auth';
import { CASES_PATH } from '../app/cases';
import { useMutationQueue } from '../app/useMutationQueue';
import controls from '../components/controls.module.css';
import { ThemeToggle } from '../components/ThemeToggle';
import { useTheme } from '../app/useTheme';
import styles from './LoginPage.module.css';

/**
 * Log ind — the only screen a signed-out browser sees (ruxd server mode,
 * phase S3). A successful login sets the session cookie and goes on to
 * `?next=` (a path on this server) or the case list. Local mode never shows
 * it: someone already signed in, or a local server, skips straight on.
 */
export function LoginPage() {
  const theme = useTheme();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [failure, setFailure] = useState<string | null>(null);
  const emailRef = useRef<HTMLInputElement>(null);
  const next = safeNext(new URLSearchParams(window.location.search).get('next'));

  // Already signed in (or a local server): no form to fill.
  useEffect(() => {
    const controller = new AbortController();
    authApi.me(controller.signal).then(
      () => {
        if (!controller.signal.aborted) window.location.replace(next);
      },
      () => emailRef.current?.focus(),
    );
    return () => controller.abort();
  }, [next]);

  const { busy, mutate } = useMutationQueue({
    scope: 'page',
    onError: (cause) => {
      setPassword('');
      setFailure(loginErrorText(cause, cause instanceof LoginError ? cause.retryAfter : undefined));
    },
  });

  const submit = (e: FormEvent) => {
    e.preventDefault();
    if (busy) return;
    setFailure(null);
    void mutate(async () => {
      await authApi.login(email.trim(), password);
      window.location.replace(next);
    });
  };

  return (
    <div className={styles.page}>
      <div className={styles.theme}>
        <ThemeToggle preference={theme.preference} setPreference={theme.setPreference} />
      </div>
      <main className={styles.card}>
        <div className={styles.brand}>
          <span className={styles.product}>
            ReUse<em className={styles.x}>X</em>
          </span>
          <span className={styles.tagline}>Ressourcekortlægning af bygninger</span>
        </div>
        <h1 className={styles.title}>Log ind</h1>
        <form className={styles.form} onSubmit={submit} noValidate>
          <label className={controls.field}>
            <span className={controls.fieldLabel}>E-mail</span>
            <input
              ref={emailRef}
              className={styles.input}
              type="email"
              name="email"
              autoComplete="username"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </label>
          <label className={controls.field}>
            <span className={controls.fieldLabel}>Adgangskode</span>
            <input
              className={styles.input}
              type="password"
              name="password"
              autoComplete="current-password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </label>
          {failure && (
            <p className={styles.error} role="alert">
              {failure}
            </p>
          )}
          <button
            type="submit"
            className={styles.submit}
            disabled={busy || email.trim() === '' || password === ''}
          >
            {busy ? 'Logger ind…' : 'Log ind'}
          </button>
        </form>
        <p className={styles.help}>
          Har du ingen konto, eller har du glemt adgangskoden? Kontakt serverens administrator.
        </p>
        {next !== CASES_PATH && <p className={styles.help}>Du sendes videre til den side, du var på.</p>}
      </main>
    </div>
  );
}

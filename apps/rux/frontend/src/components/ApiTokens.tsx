// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState, type FormEvent } from 'react';

import { authApi } from '../api/auth';
import type { ApiToken, NewApiToken } from '../api/types';
import { TOKEN_LIFETIMES, tokenErrorText, tokenLine } from '../app/auth';
import { useArmedConfirm } from '../app/useArmedConfirm';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import controls from './controls.module.css';
import styles from './ApiTokens.module.css';

/**
 * Adgangstokens — your API tokens, for scripts (`Authorization: Bearer`):
 * list, create (the token is shown once) and revoke. In the user menu.
 */
export function ApiTokens() {
  const tokens = useAsync<ApiToken[]>((s) => authApi.tokens(s), []);
  const [name, setName] = useState('');
  const [days, setDays] = useState(90);
  const [created, setCreated] = useState<NewApiToken | null>(null);
  const [failure, setFailure] = useState<string | null>(null);
  const { busy, mutate } = useMutationQueue({
    scope: 'page',
    ignoreCaseRole: true,
    onError: (cause) => setFailure(tokenErrorText(cause)),
    onSettled: () => tokens.reload(),
  });
  const confirm = useArmedConfirm<number>(busy, tokens.data);

  const create = (e: FormEvent) => {
    e.preventDefault();
    const trimmed = name.trim();
    if (!trimmed || busy) return;
    setFailure(null);
    void mutate(async () => {
      setCreated(await authApi.createToken(trimmed, days));
      setName('');
    });
  };

  return (
    <section className={styles.tokens} aria-label="Adgangstokens">
      <h3 className={styles.heading}>Adgangstokens</h3>
      {tokens.error && <p className={styles.error}>Kunne ikke hente dine tokens.</p>}
      {tokens.data && tokens.data.length === 0 && <p className={styles.muted}>Ingen tokens endnu.</p>}
      {tokens.data && tokens.data.length > 0 && (
        <ul className={styles.list}>
          {tokens.data.map((t) => (
            <li key={t.id} className={styles.item}>
              <span className={styles.name}>{t.name}</span>
              <span className={styles.meta}>{tokenLine(t)}</span>
              {confirm.armed === t.id ? (
                <button
                  type="button"
                  className={controls.btnDanger}
                  disabled={busy}
                  onBlur={confirm.disarm}
                  onClick={() => {
                    confirm.disarm();
                    setFailure(null);
                    void mutate(() => authApi.revokeToken(t.id));
                  }}
                >
                  Tilbagekald?
                </button>
              ) : (
                <button
                  type="button"
                  className={controls.btnGhost}
                  disabled={busy}
                  onClick={(e) => confirm.arm(t.id, e.currentTarget)}
                >
                  Tilbagekald
                </button>
              )}
            </li>
          ))}
        </ul>
      )}
      {created && (
        <div className={styles.created} role="status">
          <span>Kopiér tokenet nu — det vises kun denne ene gang:</span>
          <code className={styles.secret}>{created.token}</code>
        </div>
      )}
      <form className={styles.form} onSubmit={create}>
        <input
          className={controls.input}
          value={name}
          maxLength={200}
          placeholder="Navn, fx ci"
          aria-label="Navn på nyt token"
          onChange={(e) => setName(e.target.value)}
        />
        <select
          className={controls.input}
          value={days}
          aria-label="Udløber"
          onChange={(e) => setDays(Number(e.target.value))}
        >
          {TOKEN_LIFETIMES.map((l) => (
            <option key={l.days} value={l.days}>
              {l.label}
            </option>
          ))}
        </select>
        <button type="submit" className={controls.btnPrimary} disabled={busy || name.trim() === ''}>
          Opret
        </button>
      </form>
      {failure && (
        <p className={styles.error} role="alert">
          {failure}
        </p>
      )}
    </section>
  );
}

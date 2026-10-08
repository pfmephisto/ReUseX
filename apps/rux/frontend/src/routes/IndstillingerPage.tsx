// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId, useState, type FormEvent } from 'react';

import { authApi } from '../api/auth';
import { api } from '../api/client';
import type { CaseMember, CaseRole, CaseSummary } from '../api/types';
import {
  canManageMembers,
  isLastOwner,
  memberErrorText,
  ROLE_HINTS,
  ROLE_LABELS,
  ROLES,
  sortMembers,
} from '../app/auth';
import { useAuth } from '../app/AuthGate';
import { useArmedConfirm } from '../app/useArmedConfirm';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { DataTable, type Column } from '../components/DataTable';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import controls from '../components/controls.module.css';
import styles from './IndstillingerPage.module.css';

/**
 * Indstillinger — a case's settings. Today: its members (ruxd server mode,
 * phase S3). Owners add people by email, change their role and remove them;
 * everyone else sees who has access. Local mode has no users.
 */
export function IndstillingerPage() {
  const auth = useAuth();
  const cid = api.caseId ?? '';
  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Indstillinger</h1>
      </header>
      {auth?.me.mode === 'server' ? (
        <MembersPanel cid={cid} myId={auth.me.user.id} />
      ) : (
        <section className={styles.panel} aria-label="Medlemmer">
          <h2 className={styles.panelHeading}>Medlemmer</h2>
          <p className={styles.text}>
            Serveren kører i lokal tilstand: én person, intet login, og du ejer alle sager. Start ruxd uden{' '}
            <code className={styles.code}>--local</code> for brugere, roller og medlemmer.
          </p>
        </section>
      )}
    </div>
  );
}

function MembersPanel({ cid, myId }: { cid: string; myId: number }) {
  const headingId = useId();
  const info = useAsync<CaseSummary>((s) => authApi.caseInfo(cid, s), [cid]);
  const members = useAsync<CaseMember[]>((s) => authApi.members(cid, s), [cid]);
  const [email, setEmail] = useState('');
  const [role, setRole] = useState<CaseRole>('editor');
  const [failure, setFailure] = useState<string | null>(null);

  const { busy, mutate } = useMutationQueue({
    scope: 'page',
    onError: (cause) => setFailure(memberErrorText(cause)),
    onSettled: () => members.reload(),
  });
  const confirm = useArmedConfirm<number>(busy, members.data);

  if (members.error || info.error) {
    const error = (members.error ?? info.error) as Error;
    return (
      <ErrorBanner
        error={error}
        onRetry={() => {
          info.reload();
          members.reload();
        }}
        context="medlemmerne"
      />
    );
  }
  if (!members.data || !info.data) return <Spinner label="Indlæser medlemmer…" />;

  const manage = canManageMembers(info.data.role);
  const list = sortMembers(members.data);

  const add = (e: FormEvent) => {
    e.preventDefault();
    const trimmed = email.trim();
    if (!trimmed || busy) return;
    setFailure(null);
    void mutate(async () => {
      await authApi.addMember(cid, trimmed, role);
      setEmail('');
    });
  };

  const changeRole = (member: CaseMember, next: CaseRole) => {
    setFailure(null);
    void mutate(async () => {
      await authApi.setMemberRole(cid, member.user.id, next);
    });
  };

  const remove = (member: CaseMember) => {
    setFailure(null);
    confirm.disarm();
    void mutate(async () => {
      await authApi.removeMember(cid, member.user.id);
    });
  };

  const columns: Column<CaseMember>[] = [
    {
      key: 'name',
      header: 'Navn',
      render: (m) => (
        <span className={styles.name}>
          {m.user.display_name || m.user.email}
          {m.user.id === myId && <span className={styles.you}>dig</span>}
        </span>
      ),
    },
    { key: 'email', header: 'E-mail', render: (m) => <span className="mono">{m.user.email}</span> },
    {
      key: 'role',
      header: 'Rolle',
      render: (m) =>
        manage && !isLastOwner(m, list) ? (
          <select
            className={styles.select}
            value={m.role}
            aria-label={`Rolle for ${m.user.display_name || m.user.email}`}
            onChange={(e) => changeRole(m, e.target.value as CaseRole)}
          >
            {ROLES.map((r) => (
              <option key={r} value={r}>
                {ROLE_LABELS[r]}
              </option>
            ))}
          </select>
        ) : (
          <span className={styles.role} data-role={m.role} title={isLastOwner(m, list) ? 'Sagens eneste ejer' : undefined}>
            {ROLE_LABELS[m.role]}
          </span>
        ),
    },
  ];
  if (manage)
    columns.push({
      key: 'remove',
      header: '',
      render: (m) =>
        isLastOwner(m, list) ? null : confirm.armed === m.user.id ? (
          <button type="button" className={controls.btnDanger} disabled={busy} onClick={() => remove(m)} onBlur={confirm.disarm}>
            Fjern {m.user.display_name || m.user.email}?
          </button>
        ) : (
          <button
            type="button"
            className={controls.btnGhost}
            disabled={busy}
            onClick={(e) => confirm.arm(m.user.id, e.currentTarget)}
          >
            Fjern
          </button>
        ),
    });

  return (
    <section className={styles.panel} aria-labelledby={headingId}>
      <div className={styles.panelHead}>
        <h2 id={headingId} className={styles.panelHeading}>
          Medlemmer
        </h2>
        <span className={styles.count}>{list.length}</span>
        {info.data.role && <span className={styles.mine}>Din rolle: {ROLE_LABELS[info.data.role]}</span>}
      </div>
      <p className={styles.muted}>
        Ejere kan alt. Redaktører kan ændre sagen, men ikke slette den eller styre medlemmer. Læsere kan kun se.
        Administratorer har adgang til alle sager.
      </p>

      <DataTable
        columns={columns}
        rows={list}
        rowKey={(m) => String(m.user.id)}
        empty={<p className={styles.muted}>Sagen har ingen medlemmer endnu.</p>}
      />

      {manage ? (
        <form className={styles.row} onSubmit={add}>
          <label className={controls.field}>
            <span className={controls.fieldLabel}>Tilføj med e-mail</span>
            <input
              className={controls.input}
              type="email"
              value={email}
              placeholder="navn@firma.dk"
              autoComplete="off"
              onChange={(e) => setEmail(e.target.value)}
            />
          </label>
          <label className={`${controls.field} ${styles.roleField}`}>
            <span className={controls.fieldLabel}>Rolle</span>
            <select className={styles.select} value={role} onChange={(e) => setRole(e.target.value as CaseRole)}>
              {ROLES.map((r) => (
                <option key={r} value={r}>
                  {ROLE_LABELS[r]}
                </option>
              ))}
            </select>
          </label>
          <button type="submit" className={controls.btnPrimary} disabled={busy || email.trim() === ''}>
            Tilføj
          </button>
          <span className={styles.hint}>{ROLE_HINTS[role]}</span>
        </form>
      ) : (
        <p className={styles.muted}>Kun sagens ejere kan tilføje, ændre og fjerne medlemmer.</p>
      )}
      {failure && (
        <p className={styles.error} role="alert">
          {failure}
        </p>
      )}
    </section>
  );
}

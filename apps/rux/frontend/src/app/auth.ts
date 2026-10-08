// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sign-in and membership as data (ruxd server mode, phase S3): where the
 * login page sends a browser, what each refusal says, what a role may do.
 * Kept free of React so it is unit-tested in Node.
 */

import { ApiRequestError } from '../api/client';
import type { AuthMe, CaseMember, CaseRole } from '../api/types';
import { CASES_PATH } from './cases';

/** The login page. Outside every case, like the case list. */
export const LOGIN_PATH = '/login';

/** What the page should do once `GET /auth/me` has answered. */
export type AuthBoot =
  | { kind: 'proceed'; me: AuthMe }
  /** Nobody is signed in: to the login page, coming back to `next`. */
  | { kind: 'login'; href: string }
  /** The server could not answer: show the error, offer a retry. */
  | { kind: 'error' };

export function authBootAction(
  result: { ok: true; me: AuthMe } | { ok: false; status?: number },
  here: string,
): AuthBoot {
  if (result.ok) return { kind: 'proceed', me: result.me };
  if (result.status === 401) return { kind: 'login', href: loginHref(here) };
  return { kind: 'error' };
}

/** The login page, set to come back to `here` (path + query). */
export function loginHref(here: string): string {
  const next = safeNext(here);
  return next === CASES_PATH ? LOGIN_PATH : `${LOGIN_PATH}?next=${encodeURIComponent(next)}`;
}

/**
 * Where a successful login goes: `next` when it is a path on this server,
 * else the case list. Never another origin (`//evil`, `https://…`), never
 * the login page itself.
 */
export function safeNext(next: string | null | undefined): string {
  if (!next || !next.startsWith('/') || next.startsWith('//') || next.startsWith('/\\')) return CASES_PATH;
  if (next === LOGIN_PATH || next.startsWith(`${LOGIN_PATH}?`) || next.startsWith(`${LOGIN_PATH}/`))
    return CASES_PATH;
  return next;
}

/** The Danish line under a refused login. */
export function loginErrorText(cause: unknown, retryAfter?: number): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 401) return 'Forkert e-mail eller adgangskode.';
    if (cause.status === 429) {
      const wait = retryAfter ?? 60;
      return `For mange forsøg. Prøv igen om ${wait} ${wait === 1 ? 'sekund' : 'sekunder'}.`;
    }
    if (cause.status === 400) return 'Skriv både e-mail og adgangskode.';
    if (cause.status === 409) return 'Serveren kører i lokal tilstand og har ingen login.';
    if (cause.status === 403) return 'Serveren afviste forespørgslen (oprindelse ikke tilladt).';
    if (cause.status >= 500) return 'Serveren kan ikke logge ind lige nu. Prøv igen om lidt.';
  }
  return 'Kunne ikke nå serveren.';
}

/** Danish names of the roles, as the UI shows them. */
export const ROLE_LABELS: Record<CaseRole, string> = {
  owner: 'Ejer',
  editor: 'Redaktør',
  viewer: 'Læser',
};

/** What each role may do, in one line, for the role picker. */
export const ROLE_HINTS: Record<CaseRole, string> = {
  owner: 'Alt, også at slette sagen og styre medlemmer',
  editor: 'Kan ændre sagen, men ikke slette den eller styre medlemmer',
  viewer: 'Kan kun se',
};

export const ROLES: readonly CaseRole[] = ['owner', 'editor', 'viewer'];

/** Whether `role` may add, change and remove members. */
export function canManageMembers(role: CaseRole | null | undefined): boolean {
  return role === 'owner';
}

/** Whether `role` may change anything in the case. */
export function canEdit(role: CaseRole | null | undefined): boolean {
  return role === 'owner' || role === 'editor';
}

/** Members, owners first, then by name. */
export function sortMembers(members: readonly CaseMember[]): CaseMember[] {
  const rank: Record<CaseRole, number> = { owner: 0, editor: 1, viewer: 2 };
  return [...members].sort(
    (a, b) =>
      rank[a.role] - rank[b.role] ||
      (a.user.display_name || a.user.email).localeCompare(b.user.display_name || b.user.email, 'da'),
  );
}

/**
 * Whether removing or demoting `member` would leave the case without an
 * owner (the server refuses it with 409; the UI disables it first).
 */
export function isLastOwner(member: CaseMember, members: readonly CaseMember[]): boolean {
  return member.role === 'owner' && members.filter((m) => m.role === 'owner').length <= 1;
}

/** The Danish line under a refused membership change. */
export function memberErrorText(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 404 && /email/i.test(cause.message))
      return 'Der er ingen bruger med den e-mail. En administrator opretter brugere.';
    if (cause.status === 404) return 'Medlemmet findes ikke længere.';
    if (cause.status === 409 && /owner/i.test(cause.message))
      return 'En sag skal have mindst én ejer. Gør en anden til ejer først.';
    if (cause.status === 409) return 'Brugeren er allerede medlem. Skift rollen i stedet.';
    if (cause.status === 403) return 'Kun sagens ejere kan styre medlemmer.';
    if (cause.status === 400) return 'Skriv en gyldig e-mail.';
  }
  return 'Ændringen kunne ikke gemmes.';
}

/** The initials shown on the user button: "Anna Berg" → "AB". */
export function initials(name: string): string {
  const parts = name
    .replace(/@.*$/, '')
    .split(/[\s._-]+/)
    .filter(Boolean);
  if (parts.length === 0) return '?';
  const letters = parts.length === 1 ? parts[0].slice(0, 2) : parts[0][0] + parts[parts.length - 1][0];
  return letters.toUpperCase();
}

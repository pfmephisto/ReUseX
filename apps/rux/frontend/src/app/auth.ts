// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sign-in and membership as data (ruxd server mode, phase S3): where the
 * login page sends a browser, what each refusal says, what a role may do.
 * Kept free of React so it is unit-tested in Node.
 */

import { ApiRequestError } from '../api/client';
import type { ApiToken, AuthMe, CaseMember, CaseRole } from '../api/types';
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
 * Where a successful login goes: `next` when it is strictly a path on this
 * server, else the case list. A strict allowlist rather than a blocklist
 * (S3 review I1 — the URL parser strips tab and newline, so "/\t/evil.com"
 * resolved off-site):
 *  - it starts with exactly one `/`, not followed by `/` or `\`;
 *  - neither it nor its percent-decoding holds a control character or a
 *    backslash, and its decoding does not start with `//`;
 *  - resolved against this origin, it stays on this origin;
 *  - it is not the login page itself.
 */
export function safeNext(next: string | null | undefined): string {
  if (!next || !isStrictLocalPath(next)) return CASES_PATH;
  let decoded: string;
  try {
    decoded = decodeURIComponent(next);
  } catch {
    return CASES_PATH;
  }
  if (!isStrictLocalPath(decoded)) return CASES_PATH;
  const base = 'http://rux.invalid';
  let resolved: URL;
  try {
    resolved = new URL(next, base);
  } catch {
    return CASES_PATH;
  }
  if (resolved.origin !== base) return CASES_PATH;
  if (resolved.pathname === LOGIN_PATH || resolved.pathname.startsWith(`${LOGIN_PATH}/`)) return CASES_PATH;
  return next;
}

// eslint-disable-next-line no-control-regex
const CONTROL_OR_BACKSLASH = /[\u0000-\u001f\u007f\\]/;

function isStrictLocalPath(path: string): boolean {
  return /^\/(?![/\\])/.test(path) && !CONTROL_OR_BACKSLASH.test(path);
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

/** The expiry choices for a new API token (days; 0 = never). */
export const TOKEN_LIFETIMES: readonly { days: number; label: string }[] = [
  { days: 30, label: '30 dage' },
  { days: 90, label: '90 dage' },
  { days: 365, label: '1 år' },
  { days: 0, label: 'Udløber aldrig' },
];

/** A token's line under its name: scope, expiry and last use, in Danish. */
export function tokenLine(t: ApiToken): string {
  const day = (iso: string) => iso.slice(0, 10);
  return [
    t.case ? `kun sagen ${t.case}` : 'alle dine sager',
    t.expires_at ? `udløber ${day(t.expires_at)}` : 'udløber aldrig',
    t.last_used_at ? `brugt ${day(t.last_used_at)}` : 'aldrig brugt',
  ].join(' · ');
}

/** The Danish line under a refused token change. */
export function tokenErrorText(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 400) return 'Giv tokenet et navn.';
    if (cause.status === 403) return 'Tokens styres fra en logget-ind session.';
    if (cause.status === 404) return 'Tokenet findes ikke længere.';
  }
  return 'Ændringen kunne ikke gemmes.';
}

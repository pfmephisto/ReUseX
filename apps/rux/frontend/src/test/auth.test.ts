// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { AuthClient, LoginError, type AuthFetch } from '../api/auth';
import { ApiRequestError } from '../api/client';
import type { AuthMe, CaseMember } from '../api/types';
import {
  authBootAction,
  canEdit,
  canManageMembers,
  initials,
  isLastOwner,
  LOGIN_PATH,
  loginErrorText,
  loginHref,
  memberErrorText,
  ROLE_LABELS,
  safeNext,
  sortMembers,
  tokenLine,
} from '../app/auth';
import { editVisible } from '../app/CaseRoleContext';
import { shouldSendToLogin } from '../app/unauthorized';

const me: AuthMe = {
  mode: 'server',
  via: 'session',
  user: { id: 1, email: 'a@x.dk', display_name: 'Anna', is_admin: false, disabled: false, created_at: '' },
};

const member = (id: number, name: string, role: CaseMember['role']): CaseMember => ({
  user: { id, email: `${name.toLowerCase()}@x.dk`, display_name: name },
  role,
});

describe('auth boot (S3)', () => {
  it('proceeds when signed in, goes to the login page on 401, else shows an error', () => {
    expect(authBootAction({ ok: true, me }, '/sager')).toEqual({ kind: 'proceed', me });
    expect(authBootAction({ ok: false, status: 401 }, '/sager/kontor/kortlaegning?type=3')).toEqual({
      kind: 'login',
      href: `${LOGIN_PATH}?next=${encodeURIComponent('/sager/kontor/kortlaegning?type=3')}`,
    });
    expect(authBootAction({ ok: false, status: 401 }, '/sager')).toEqual({ kind: 'login', href: LOGIN_PATH });
    expect(authBootAction({ ok: false, status: 503 }, '/sager')).toEqual({ kind: 'error' });
    expect(authBootAction({ ok: false }, '/sager')).toEqual({ kind: 'error' });
  });

  it('only ever sends a login on to a path on this server', () => {
    expect(safeNext('/sager/kontor')).toBe('/sager/kontor');
    expect(safeNext(null)).toBe('/sager');
    expect(safeNext('')).toBe('/sager');
    expect(safeNext('https://evil.example/')).toBe('/sager');
    expect(safeNext('//evil.example/x')).toBe('/sager');
    expect(safeNext('/\\evil.example')).toBe('/sager');
    expect(safeNext('/login')).toBe('/sager');
    expect(safeNext('/login?next=/x')).toBe('/sager');
    expect(loginHref('/sager')).toBe('/login');
  });

  it('says why a login was refused, in Danish', () => {
    expect(loginErrorText(new ApiRequestError(401, 'wrong', '/x'))).toBe('Forkert e-mail eller adgangskode.');
    expect(loginErrorText(new ApiRequestError(429, 'slow', '/x'), 42)).toContain('42 sekunder');
    expect(loginErrorText(new ApiRequestError(429, 'slow', '/x'), 1)).toContain('1 sekund.');
    expect(loginErrorText(new ApiRequestError(409, 'local', '/x'))).toContain('lokal tilstand');
    expect(loginErrorText(new ApiRequestError(503, 'db', '/x'))).toContain('lige nu');
    expect(loginErrorText(new TypeError('network'))).toBe('Kunne ikke nå serveren.');
  });
});

describe('roles and members (S3)', () => {
  it('lets owners manage members and editors edit', () => {
    expect(canManageMembers('owner')).toBe(true);
    expect(canManageMembers('editor')).toBe(false);
    expect(canManageMembers('viewer')).toBe(false);
    expect(canManageMembers(null)).toBe(false);
    expect(canEdit('editor')).toBe(true);
    expect(canEdit('viewer')).toBe(false);
    expect(ROLE_LABELS.viewer).toBe('Læser');
  });

  it('sorts owners first, then by name, and spots the last owner', () => {
    const list = [member(3, 'Ørsted', 'viewer'), member(2, 'Bo', 'owner'), member(4, 'Anna', 'editor')];
    expect(sortMembers(list).map((m) => m.user.id)).toEqual([2, 4, 3]);
    expect(isLastOwner(list[1], list)).toBe(true);
    expect(isLastOwner(list[0], list)).toBe(false);
    const two = [...list, member(5, 'Cai', 'owner')];
    expect(isLastOwner(list[1], two)).toBe(false);
  });

  it('turns membership refusals into Danish', () => {
    expect(memberErrorText(new ApiRequestError(404, "no user has the email 'x'", '/m'))).toContain('ingen bruger');
    expect(memberErrorText(new ApiRequestError(409, 'a case must keep at least one owner', '/m'))).toContain(
      'mindst én ejer',
    );
    expect(memberErrorText(new ApiRequestError(409, "'x' is already a member", '/m'))).toContain('allerede medlem');
    expect(memberErrorText(new ApiRequestError(403, 'role', '/m'))).toContain('ejere');
  });

  it('makes initials for the user button', () => {
    expect(initials('Anna Berg')).toBe('AB');
    expect(initials('anna.berg@x.dk')).toBe('AB');
    expect(initials('root')).toBe('RO');
    expect(initials('')).toBe('?');
  });
});

describe('AuthClient (S3)', () => {
  const recorder = (status: number, body: unknown, headers: Record<string, string> = {}) => {
    const calls: { url: string; method?: string; body?: string }[] = [];
    const doFetch: AuthFetch = async (url, init) => {
      calls.push({ url, method: init?.method, body: init?.body });
      return new Response(status === 204 ? null : JSON.stringify(body), { status, headers });
    };
    return { calls, client: new AuthClient({ fetch: doFetch }) };
  };

  it('posts the login as JSON and returns who signed in', async () => {
    const { calls, client } = recorder(200, me);
    await expect(client.login('a@x.dk', 'pw')).resolves.toEqual(me);
    expect(calls[0]).toEqual({
      url: '/api/v1/auth/login',
      method: 'POST',
      body: JSON.stringify({ email: 'a@x.dk', password: 'pw' }),
    });
  });

  it('carries Retry-After on a rate-limited login', async () => {
    const { client } = recorder(429, { error: 'too many' }, { 'Retry-After': '17' });
    const error = await client.login('a@x.dk', 'pw').catch((e: unknown) => e);
    expect(error).toBeInstanceOf(LoginError);
    expect((error as LoginError).status).toBe(429);
    expect((error as LoginError).retryAfter).toBe(17);
  });

  it('addresses members under the case', async () => {
    const { calls, client } = recorder(204, null);
    await client.removeMember('kontor', 7);
    await client.setMemberRole('kontor', 7, 'viewer').catch(() => undefined);
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'DELETE /api/v1/cases/kontor/members/7',
      'PATCH /api/v1/cases/kontor/members/7',
    ]);
  });
});

describe('safeNext bypass corpus (S3 review I1)', () => {
  const offsite = [
    '/\t/evil.com',
    '/\n/evil.com',
    '/\r/evil.com',
    '/%09/evil.com',
    '/%0a/evil.com',
    '/%2F/evil.com',
    '/%2f%2fevil.com',
    '/%5Cevil.com',
    '/\\evil.com',
    '\\/evil.com',
    '//evil.com',
    '///evil.com',
    'https://evil.com/',
    'http:evil.com',
    'javascript:alert(1)',
    ' /sager',
    '/sager\u0000x',
    '/%00',
    '%2F%2Fevil.com',
    '/%E0%A4%A',
    '/login',
    '/login?next=/x',
    '/login/',
  ];
  for (const next of offsite)
    it(`refuses ${JSON.stringify(next)}`, () => {
      expect(safeNext(next)).toBe('/sager');
    });

  it('keeps ordinary in-app paths with their query and fragment', () => {
    expect(safeNext('/sager/kontor/kortlaegning?type=3#x')).toBe('/sager/kontor/kortlaegning?type=3#x');
    expect(safeNext('/sager/b%C3%B8gevej')).toBe('/sager/b%C3%B8gevej');
    expect(safeNext('/')).toBe('/');
  });
});

describe('session ended mid-use (S3 review M9)', () => {
  it('sends a 401 to the login page in server mode only, except from the gate and the form', () => {
    expect(shouldSendToLogin(401, '/api/v1/cases/k/project', true)).toBe(true);
    expect(shouldSendToLogin(401, '/api/v1/cases/k/project', false)).toBe(false);
    expect(shouldSendToLogin(403, '/api/v1/cases/k/project', true)).toBe(false);
    expect(shouldSendToLogin(401, '/api/v1/auth/me', true)).toBe(false);
    expect(shouldSendToLogin(401, '/api/v1/auth/login', true)).toBe(false);
    expect(shouldSendToLogin(401, '/api/v1/auth/tokens', true)).toBe(true);
  });

  it('shows edit controls to editors and owners, hides them from viewers', () => {
    expect(editVisible('owner')).toBe(true);
    expect(editVisible('editor')).toBe(true);
    expect(editVisible('viewer')).toBe(false);
    expect(editVisible(null)).toBe(false);
    expect(editVisible(undefined)).toBe(true); // not known yet: no flicker
  });
});

describe('API tokens (S3 review I5)', () => {
  it('describes a token without ever showing it', () => {
    expect(
      tokenLine({
        id: 1,
        name: 'ci',
        case: 'kontor',
        created_at: '2026-10-08T10:00:00Z',
        expires_at: '2027-01-06T10:00:00Z',
        last_used_at: null,
      }),
    ).toBe('kun sagen kontor · udløber 2027-01-06 · aldrig brugt');
    expect(
      tokenLine({ id: 2, name: 'x', case: null, created_at: '', expires_at: null, last_used_at: '2026-10-09T00:00:00Z' }),
    ).toBe('alle dine sager · udløber aldrig · brugt 2026-10-09');
  });
});

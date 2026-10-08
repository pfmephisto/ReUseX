// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sign-in and case membership (`docs/gui/openapi.yaml`, tags `auth` and
 * `members`; ruxd server mode, phase S3).
 *
 * The session is an HttpOnly cookie the server sets on login: this client
 * never sees the token, and `<img>`, raw `fetch` and the events socket carry
 * it with no extra code. In local mode (`ruxd --local`) `me()` answers with
 * the implicit user and there is no login at all.
 */

import { ApiRequestError, DEFAULT_BASE_URL, describeFailure } from './client';
import type { AuthMe, CaseMember, CaseRole, CaseSummary } from './types';

export type AuthFetch = (
  input: string,
  init?: { method?: string; headers?: Record<string, string>; body?: string; signal?: AbortSignal },
) => Promise<Response>;

/** A refused login, with the server's wait for a rate-limited one. */
export class LoginError extends ApiRequestError {
  constructor(
    status: number,
    message: string,
    url: string,
    /** Seconds from `Retry-After` (429), else undefined. */
    readonly retryAfter?: number,
  ) {
    super(status, message, url);
  }
}

export class AuthClient {
  private readonly baseUrl: string;
  private readonly doFetch: AuthFetch;

  constructor(options: { baseUrl?: string; fetch?: AuthFetch } = {}) {
    this.baseUrl = (options.baseUrl ?? DEFAULT_BASE_URL).replace(/\/+$/, '');
    this.doFetch = options.fetch ?? ((input, init) => fetch(input, { ...init, credentials: 'same-origin' }));
  }

  private async send<T>(method: string, path: string, body?: unknown, signal?: AbortSignal): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const response = await this.doFetch(url, {
      method,
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal,
    });
    if (!response.ok) throw new ApiRequestError(response.status, await describeFailure(response), url);
    return (response.status === 204 ? undefined : await response.json()) as T;
  }

  /** Who this browser is signed in as. Rejects with a 401 when nobody. */
  me(signal?: AbortSignal): Promise<AuthMe> {
    return this.send<AuthMe>('GET', '/auth/me', undefined, signal);
  }

  async login(email: string, password: string): Promise<AuthMe> {
    const url = `${this.baseUrl}/auth/login`;
    const response = await this.doFetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
    });
    if (!response.ok) {
      const wait = Number(response.headers.get('Retry-After'));
      throw new LoginError(
        response.status,
        await describeFailure(response),
        url,
        Number.isFinite(wait) && wait > 0 ? wait : undefined,
      );
    }
    return (await response.json()) as AuthMe;
  }

  logout(): Promise<void> {
    return this.send<void>('POST', '/auth/logout', {});
  }

  /** One case, with the caller's role in it. */
  caseInfo(cid: string, signal?: AbortSignal): Promise<CaseSummary> {
    return this.send<CaseSummary>('GET', `/cases/${encodeURIComponent(cid)}`, undefined, signal);
  }

  async members(cid: string, signal?: AbortSignal): Promise<CaseMember[]> {
    const list = await this.send<{ members: CaseMember[] }>(
      'GET',
      `/cases/${encodeURIComponent(cid)}/members`,
      undefined,
      signal,
    );
    return list.members;
  }

  addMember(cid: string, email: string, role: CaseRole): Promise<CaseMember> {
    return this.send<CaseMember>('POST', `/cases/${encodeURIComponent(cid)}/members`, { email, role });
  }

  setMemberRole(cid: string, userId: number, role: CaseRole): Promise<CaseMember> {
    return this.send<CaseMember>('PATCH', `/cases/${encodeURIComponent(cid)}/members/${userId}`, { role });
  }

  removeMember(cid: string, userId: number): Promise<void> {
    return this.send<void>('DELETE', `/cases/${encodeURIComponent(cid)}/members/${userId}`);
  }
}

/** The client the shell, the login page and Indstillinger use. */
export const authApi = new AuthClient();

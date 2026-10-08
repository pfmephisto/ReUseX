// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A session that ends while a page is open (it expired, someone logged it
 * out, an administrator disabled the account): the next API answer is a
 * 401, and the page goes back to the login page — which comes back here —
 * instead of showing errors (S3 review M9).
 *
 * Only in server mode: AuthGate switches it on once `GET /auth/me` says so.
 * A local server with `--auth-token` also answers 401, but has no login page
 * to go to.
 */

import { loginHref } from './auth';

let enabled = false;
let leaving = false;

/** Turn the redirect on (server mode) or off. */
export function setLoginRedirect(on: boolean): void {
  enabled = on;
}

/** Whether a response with @p status from @p url means "sign in again". */
export function shouldSendToLogin(status: number, url: string, on: boolean = enabled): boolean {
  if (!on || status !== 401) return false;
  // The gate and the login form deal with these themselves.
  return !/\/auth\/(me|login)(\?|$)/.test(url);
}

/** `fetch`, plus the redirect above. The default transport of every client. */
export async function authAwareFetch(input: string, init?: RequestInit): Promise<Response> {
  const response = await fetch(input, init);
  if (shouldSendToLogin(response.status, input) && !leaving) {
    leaving = true;
    window.location.replace(loginHref(window.location.pathname + window.location.search));
  }
  return response;
}

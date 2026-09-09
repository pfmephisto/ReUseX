// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { ErrorBanner } from 'reusex-gui';

const noop = () => {};

// `ApiRequestError` (the class whose `isRetryable` / `isNotImplemented` /
// `isNotFound` getters drive ErrorBanner's message branching) is an internal
// type of the api client module and is not re-exported from the package
// index — previews may only import from 'reusex-gui'. So these cells sweep
// the plain-`Error` fallback path (network failure, `error.message` passed
// straight through) rather than the three status-code branches. See
// wave-a.md for the note.

/** Network failure while loading the project summary, with a retry action. */
export const NetworkFailureWithRetry = () => (
  <ErrorBanner
    error={new Error('Failed to fetch: the server at localhost:8080 refused the connection')}
    context="the project summary"
    onRetry={noop}
  />
);

/** Same failure, no retry handler supplied — no button rendered. */
export const NetworkFailureNoRetry = () => (
  <ErrorBanner
    error={new Error('Failed to fetch: the server at localhost:8080 refused the connection')}
    context="sensor frame 0412"
  />
);

/** No `context` prop — falls back to the generic "this data" subject. */
export const NoContext = () => (
  <ErrorBanner error={new Error('Unexpected token < in JSON at position 0')} onRetry={noop} />
);

/** Loading the point cloud page for a large fused cloud, mid-decode error. */
export const CloudPointsFailure = () => (
  <ErrorBanner
    error={new Error('Failed to parse binary point page: unexpected end of stream')}
    context="cloud points for `cloud`"
    onRetry={noop}
  />
);

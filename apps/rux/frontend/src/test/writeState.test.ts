// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import { describeWriteFailure, shouldReloadAfter } from '../data/writeState';

const url = '/api/v1/materials/mp-0001';

function failure(status: number, message = 'server said so') {
  return describeWriteFailure(new ApiRequestError(status, message, url), 'this passport');
}

describe('describeWriteFailure', () => {
  it('separates a job conflict from a busy lock, which need opposite advice', () => {
    // This is the whole point of the function. 409 means a stage holds the
    // writer lock for minutes, so "try again" is advice that cannot work; 503
    // means it was busy for an instant and retrying is exactly right.
    const conflict = failure(409);
    const busy = failure(503);

    expect(conflict.kind).toBe('conflict');
    expect(conflict.retryable).toBe(false);
    expect(conflict.detail).toMatch(/wait for the running stage/i);

    expect(busy.kind).toBe('busy');
    expect(busy.retryable).toBe(true);
    expect(busy.detail).toMatch(/send it again/i);
  });

  it('always says that nothing was written', () => {
    // Both server-side checks fail before opening a write transaction, so the
    // user must never be left wondering whether a partial edit landed.
    for (const status of [409, 503]) {
      expect(failure(status).detail).toMatch(/not changed|nothing was written/i);
    }
  });

  it('passes a 400 through verbatim rather than paraphrasing it', () => {
    // The server's wording names the offending label or property; nothing this
    // function could invent would be as specific.
    const message =
      "label 99 is not defined for cloud 'labels'; this renames existing classes rather than creating new ones";
    const described = failure(400, message);
    expect(described.kind).toBe('rejected');
    expect(described.detail).toBe(message);
    expect(described.retryable).toBe(false);
  });

  it('keeps the server wording for a refusal it could not have guessed', () => {
    const message =
      "label names of the 'instances' cloud encode the semantic class and instance id";
    expect(failure(409, message).serverMessage).toBe(message);
  });

  it('treats a 404 as the resource having gone, and asks for a reload', () => {
    const described = failure(404);
    expect(described.kind).toBe('missing');
    expect(shouldReloadAfter(described)).toBe(true);
  });

  it('names the content-type gate when a proxy strips the header', () => {
    expect(failure(415).kind).toBe('unsupported');
  });

  it('falls back for a plain Error with no status at all', () => {
    const described = describeWriteFailure(new Error('network died'), 'this passport');
    expect(described.kind).toBe('unknown');
    expect(described.detail).toBe('network died');
  });

  it('names the subject it was given, so two panes do not share wording', () => {
    const legend = describeWriteFailure(
      new ApiRequestError(409, 'busy', url),
      'this label legend',
    );
    expect(legend.detail).toMatch(/this label legend/);
  });
});

describe('shouldReloadAfter', () => {
  it('does not refetch after a conflict or a busy lock', () => {
    // Neither changed anything, so the pane's data is still current and a
    // refetch would only cost a request.
    expect(shouldReloadAfter(failure(409))).toBe(false);
    expect(shouldReloadAfter(failure(503))).toBe(false);
    expect(shouldReloadAfter(failure(400))).toBe(false);
  });
});

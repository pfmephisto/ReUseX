// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The server-level case routes (`docs/gui/openapi.yaml`, tag `cases`): the case
 * list, creating, renaming and deleting a case, and uploading a `.rux` as a new
 * one. Everything *inside* a case goes through `RuxApiClient` (`client.ts`).
 *
 * Uploads are chunked: ruxd buffers each request body in memory, so a
 * multi-GB scan goes up as a series of `PUT /uploads/{id}?offset=` chunks no
 * larger than the server's `chunk_bytes`, then `POST /uploads/{id}/complete`
 * turns the staged file into a case.
 */

import { uploadChunks } from '../app/cases';
import { ApiRequestError, DEFAULT_BASE_URL, describeFailure } from './client';
import type { CaseList, CaseSummary, UploadSession } from './types';

/** The transport, injectable for tests. Bodies may be binary (a `Blob`). */
export type CasesFetch = (
  input: string,
  init?: { method?: string; headers?: Record<string, string>; body?: string | Blob; signal?: AbortSignal },
) => Promise<Response>;

export interface CaseUploadProgress {
  sent: number;
  total: number;
}

/** What uploadFile needs of a file: its size, and a slice of its bytes. */
export interface UploadSource {
  size: number;
  slice(start: number, end: number): Blob;
}

export class CasesClient {
  private readonly baseUrl: string;
  private readonly doFetch: CasesFetch;

  constructor(options: { baseUrl?: string; fetch?: CasesFetch } = {}) {
    this.baseUrl = (options.baseUrl ?? DEFAULT_BASE_URL).replace(/\/+$/, '');
    this.doFetch = options.fetch ?? ((input, init) => fetch(input, init));
  }

  private async send<T>(method: string, path: string, body?: unknown, signal?: AbortSignal): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const response = await this.doFetch(url, {
      method,
      // Every mutating route requires it (the server's CSRF floor).
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal,
    });
    if (!response.ok) throw new ApiRequestError(response.status, await describeFailure(response), url);
    return (response.status === 204 ? undefined : await response.json()) as T;
  }

  list(signal?: AbortSignal): Promise<CaseList> {
    return this.send<CaseList>('GET', '/cases', undefined, signal);
  }

  create(name: string): Promise<CaseSummary> {
    return this.send<CaseSummary>('POST', '/cases', { name });
  }

  update(cid: string, patch: { name?: string; archived?: boolean }): Promise<CaseSummary> {
    return this.send<CaseSummary>('PATCH', `/cases/${encodeURIComponent(cid)}`, patch);
  }

  /** Moves the case to the server's trash; refused (409) while a job runs. */
  remove(cid: string): Promise<void> {
    return this.send<void>('DELETE', `/cases/${encodeURIComponent(cid)}`);
  }

  /**
   * Upload `file` as a new case named `name`, chunk by chunk, reporting
   * progress after each chunk. Aborting abandons the upload on the server too.
   */
  async uploadFile(
    file: UploadSource,
    name: string,
    onProgress?: (p: CaseUploadProgress) => void,
    signal?: AbortSignal,
  ): Promise<CaseSummary> {
    const session = await this.send<UploadSession>('POST', '/uploads', { name, size: file.size }, signal);
    const id = encodeURIComponent(session.id);
    try {
      onProgress?.({ sent: session.received, total: session.size });
      for (const chunk of uploadChunks(session.size, session.chunk_bytes, session.received)) {
        const url = `${this.baseUrl}/uploads/${id}?offset=${chunk.start}`;
        const response = await this.doFetch(url, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/octet-stream' },
          body: file.slice(chunk.start, chunk.end),
          signal,
        });
        if (!response.ok) throw new ApiRequestError(response.status, await describeFailure(response), url);
        onProgress?.({ sent: chunk.end, total: session.size });
      }
      return await this.send<CaseSummary>('POST', `/uploads/${id}/complete`, {}, signal);
    } catch (cause) {
      // Best effort: free the staging file. The server also expires it.
      void this.doFetch(`${this.baseUrl}/uploads/${id}`, { method: 'DELETE' }).catch(() => undefined);
      throw cause;
    }
  }
}

/** The client the case list uses. Same-origin, default base path. */
export const casesApi = new CasesClient();

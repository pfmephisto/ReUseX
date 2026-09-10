// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Tests for the Gaussian-splat layer's non-WebGL half (#322).
 *
 * The renderer itself needs a GPU and a canvas, so what is pinned here is
 * everything the user's experience actually turns on: which URL is fetched,
 * what a splat is described as, and what a project *without* one is told.
 * `SplatScene.ts` — the three.js glue — is out of scope by construction; that
 * split is why `gsplatLayer.ts` exists as a separate module.
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient, type FetchLike } from '../api/client';
import type { GsplatInfo } from '../api/types';
import { describeGsplat, formatSplatSize, gsplatNote } from '../viewport/gsplatLayer';

const SPLAT: GsplatInfo = {
  name: 'splat',
  format: 'ply',
  gaussian_count: 1_200_000,
  sh_degree: 3,
  byte_size: 297 * 1024 * 1024,
  stage: 'gsplat',
  created_at: '2026-09-10 08:00:00',
};

/** A stub transport that answers `payload`, recording the URLs it was given. */
function stubFetch(payload: unknown) {
  const urls: string[] = [];
  const fetchLike: FetchLike = (url) => {
    urls.push(url);
    return Promise.resolve(
      new Response(JSON.stringify(payload), {
        status: 200,
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { urls, fetchLike };
}

describe('RuxApiClient gsplat routes', () => {
  it('Gsplats_StoredSplats_RequestsTheContractPathAndUnwrapsTheEnvelope', async () => {
    const { urls, fetchLike } = stubFetch({ gsplats: [SPLAT] });
    const client = new RuxApiClient({ fetch: fetchLike });

    const splats = await client.gsplats();

    expect(urls).toEqual(['/api/v1/gsplats']);
    expect(splats).toHaveLength(1);
    expect(splats[0].gaussian_count).toBe(1_200_000);
  });

  it('Gsplats_ProjectWithoutSplats_ResolvesToAnEmptyArray', async () => {
    // The normal state of most projects. A client that treated it as an error
    // would put a red banner on "you have not run the trainer yet".
    const { fetchLike } = stubFetch({ gsplats: [] });
    const client = new RuxApiClient({ fetch: fetchLike });

    expect(await client.gsplats()).toEqual([]);
  });

  it('Gsplat_NamedSplat_EncodesTheNameIntoThePath', async () => {
    const { urls, fetchLike } = stubFetch(SPLAT);
    const client = new RuxApiClient({ fetch: fetchLike });

    await client.gsplat('run 2');

    expect(urls).toEqual(['/api/v1/gsplats/run%202']);
  });

  it('GsplatDataUrl_DefaultBase_IsTheSameOriginBinaryPath', () => {
    // The splat loader fetches this itself, so it must be usable verbatim.
    expect(new RuxApiClient().gsplatDataUrl('splat')).toBe('/api/v1/gsplats/splat/data');
  });

  it('GsplatDataUrl_NameNeedingEscaping_IsEncoded', () => {
    expect(new RuxApiClient().gsplatDataUrl('run 2/x')).toBe(
      '/api/v1/gsplats/run%202%2Fx/data',
    );
  });

  it('GsplatDataUrl_ExplicitBaseUrl_KeepsTheConfiguredHost', () => {
    const client = new RuxApiClient({ baseUrl: 'http://127.0.0.1:8420/api/v1' });
    expect(client.gsplatDataUrl('splat')).toBe(
      'http://127.0.0.1:8420/api/v1/gsplats/splat/data',
    );
  });
});

describe('describeGsplat', () => {
  it('DescribeGsplat_FullMetadata_ReportsCountSizeAndDegree', () => {
    const text = describeGsplat(SPLAT);
    expect(text).toContain('Gaussians');
    expect(text).toContain('297 MB');
    expect(text).toContain('SH degree 3');
  });

  it('DescribeGsplat_ViewIndependentColour_SaysFlatColourNotDegreeZero', () => {
    expect(describeGsplat({ ...SPLAT, sh_degree: 0 })).toContain('flat colour');
  });

  it('DescribeGsplat_MissingField_OmitsItRatherThanPrintingUndefined', () => {
    // Tolerant on purpose: this renders whatever the server actually sent, and
    // a row reading `undefined Gaussians` is worse than a shorter row.
    const partial = { name: 'splat', format: 'ply' } as GsplatInfo;
    expect(describeGsplat(partial)).toBe('');
    expect(describeGsplat({ ...partial, gaussian_count: 10 })).not.toContain('undefined');
  });
});

describe('formatSplatSize', () => {
  it('FormatSplatSize_ByteMagnitudes_PicksTheReadableUnit', () => {
    expect(formatSplatSize(512)).toBe('512 B');
    expect(formatSplatSize(4096)).toBe('4 kB');
    expect(formatSplatSize(5 * 1024 * 1024)).toBe('5 MB');
    expect(formatSplatSize(3 * 1024 * 1024 * 1024)).toBe('3.0 GB');
  });

  it('FormatSplatSize_AbsentOrNonsense_IsNull', () => {
    expect(formatSplatSize(undefined)).toBeNull();
    expect(formatSplatSize(Number.NaN)).toBeNull();
    expect(formatSplatSize(-1)).toBeNull();
  });
});

describe('gsplatNote', () => {
  it('GsplatNote_StoredSplats_HasNothingToSay', () => {
    // The rows speak for themselves; a note above them would be noise.
    expect(gsplatNote([SPLAT], null)).toBeNull();
  });

  it('GsplatNote_ProjectWithoutSplats_NamesBothWaysToGetOne', () => {
    const note = gsplatNote([], null);
    expect(note).toContain('rux create gsplat');
    expect(note).toContain('rux import gsplat');
  });

  it('GsplatNote_RequestFailed_ReportsTheTransportErrorNotAnEmptyProject', () => {
    // Saying "this project has no splat" because the request never arrived
    // would state something the client does not know.
    const note = gsplatNote([], new Error('project database is busy'));
    expect(note).toContain('project database is busy');
    expect(note).not.toContain('rux create gsplat');
  });

  it('GsplatNote_ListNotYetLoaded_IsNull', () => {
    expect(gsplatNote(null, null)).toBeNull();
  });
});

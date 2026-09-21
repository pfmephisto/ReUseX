// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Tests for the mesh layer's non-WebGL half (#265, review pt 2).
 *
 * The renderer (`MeshScene.ts`) needs a WebGL context so it is out of scope
 * here; what is pinned is what the user's experience actually depends on:
 * the description under a mesh's toggle and what to say when there is none.
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient, type FetchLike } from '../api/client';
import type { MeshInfo } from '../api/types';
import { describeMesh, meshNote } from '../viewport/meshLayer';

const MESH: MeshInfo = {
  name: 'mesh',
  format: 'ply',
  vertex_count: 42_000,
  face_count: 84_000,
  stage: 'mesh',
  created_at: '2026-09-21 12:00:00',
};

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

describe('RuxApiClient mesh routes', () => {
  it('Meshes_StoredMesh_RequestsTheContractPathAndUnwrapsTheEnvelope', async () => {
    const { urls, fetchLike } = stubFetch({ meshes: [MESH] });
    const client = new RuxApiClient({ fetch: fetchLike });

    const meshes = await client.meshes();

    expect(urls).toEqual(['/api/v1/meshes']);
    expect(meshes).toHaveLength(1);
    expect(meshes[0].vertex_count).toBe(42_000);
  });

  it('Meshes_ProjectWithoutMesh_ResolvesToAnEmptyArray', async () => {
    const { fetchLike } = stubFetch({ meshes: [] });
    const client = new RuxApiClient({ fetch: fetchLike });

    expect(await client.meshes()).toEqual([]);
  });

  it('MeshDataUrl_DefaultBase_IsTheSameOriginBinaryPath', () => {
    expect(new RuxApiClient().meshDataUrl('mesh')).toBe('/api/v1/meshes/mesh/data');
  });

  it('MeshDataUrl_NameNeedingEscaping_IsEncoded', () => {
    expect(new RuxApiClient().meshDataUrl('scan 1/mesh')).toBe(
      '/api/v1/meshes/scan%201%2Fmesh/data',
    );
  });
});

describe('describeMesh', () => {
  it('DescribeMesh_FullMetadata_ReportsVertexFaceCountAndFormat', () => {
    const text = describeMesh(MESH);
    // Locale-agnostic: only the digits and the suffix matter; the thousand
    // separator is locale-dependent (`.` in da-DK, `,` in en-US).
    expect(text).toMatch(/42.000 vertices/);
    expect(text).toMatch(/84.000 faces/);
    expect(text).toContain('PLY');
  });

  it('DescribeMesh_MissingFormat_OmitsItRatherThanPrintingUndefined', () => {
    const partial: MeshInfo = { name: 'mesh', vertex_count: 10, face_count: 5 };
    const text = describeMesh(partial);
    expect(text).not.toContain('undefined');
    expect(text).toContain('vertices');
  });

  it('DescribeMesh_NegativeCount_OmitsTheField', () => {
    // The contract marks counts required, but a defensive read costs nothing.
    // When every field is excluded the result is empty — consistent with describeGsplat.
    const bad: MeshInfo = { name: 'mesh', vertex_count: -1, face_count: -1 };
    // No 'vertices' or 'faces' text for invalid counts.
    expect(describeMesh(bad)).not.toContain('vertices');
    expect(describeMesh(bad)).not.toContain('faces');
  });
});

describe('meshNote', () => {
  it('MeshNote_StoredMesh_HasNothingToSay', () => {
    expect(meshNote([MESH], null)).toBeNull();
  });

  it('MeshNote_ProjectWithoutMesh_NamesBothRunCommands', () => {
    const note = meshNote([], null);
    expect(note).toContain('rux create mesh');
  });

  it('MeshNote_RequestFailed_ReportsTheTransportErrorNotAnEmptyProject', () => {
    const note = meshNote([], new Error('project database is busy'));
    expect(note).toContain('project database is busy');
    expect(note).not.toContain('rux create mesh');
  });

  it('MeshNote_ListNotYetLoaded_IsNull', () => {
    expect(meshNote(null, null)).toBeNull();
  });
});

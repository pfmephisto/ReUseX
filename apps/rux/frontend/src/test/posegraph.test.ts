// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Tests for the pose-graph layer's non-WebGL half (#265, review pt 4).
 *
 * The Three.js scene (`PoseGraphScene.ts`) needs a GPU, so that is out of
 * scope here.  What is pinned is the pure-logic module (`posegraphLayer.ts`):
 * colour mapping, residual statistics, note text.
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient, type FetchLike } from '../api/client';
import type { PoseGraph, PoseGraphEdge } from '../api/types';
import {
  EDGE_BASE_COLORS,
  computeResidualStats,
  posegraphNote,
  residualColor,
} from '../viewport/posegraphLayer';

// ----------------------------------------------------------------- fixtures --

const EDGES: PoseGraphEdge[] = [
  { from: 1, to: 2, type: 'odometry', residual: 0.01 },
  { from: 2, to: 3, type: 'odometry', residual: 0.05 },
  { from: 1, to: 10, type: 'loop_closure', residual: 0.40 },
  { from: 5, to: 20, type: 'panorama', residual: 0.20 },
];

const GRAPH: PoseGraph = {
  nodes: [
    { id: 1, pose: Array(16).fill(0).map((_, i) => (i % 5 === 0 ? 1 : 0)) },
    { id: 2, pose: Array(16).fill(0).map((_, i) => (i % 5 === 0 ? 1 : 0)) },
    { id: 3, pose: Array(16).fill(0).map((_, i) => (i % 5 === 0 ? 1 : 0)) },
  ],
  edges: EDGES,
};

// ----------------------------------------------------------- API client route --

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

describe('RuxApiClient posegraph route', () => {
  it('Posegraph_HitsTheContractPath', async () => {
    const { urls, fetchLike } = stubFetch(GRAPH);
    const client = new RuxApiClient({ fetch: fetchLike });
    const result = await client.posegraph();
    expect(urls[0]).toContain('/posegraph');
    expect(result.nodes).toHaveLength(3);
    expect(result.edges).toHaveLength(4);
  });
});

// ---------------------------------------------------------- residual stats --

describe('computeResidualStats', () => {
  it('ComputesMinMaxPerType', () => {
    const stats = computeResidualStats(EDGES);
    expect(stats.odometry.min).toBeCloseTo(0.01);
    expect(stats.odometry.max).toBeCloseTo(0.05);
    expect(stats.loop_closure.min).toBeCloseTo(0.40);
    expect(stats.loop_closure.max).toBeCloseTo(0.40 + 0.001); // expanded span
    expect(stats.panorama.min).toBeCloseTo(0.20);
  });

  it('EmptyEdgeSet_ReturnsDefaultSpan', () => {
    const stats = computeResidualStats([]);
    expect(stats.odometry.min).toBe(0);
    expect(stats.odometry.max).toBe(1);
  });

  it('SingleEdge_ExpandsMinMaxSoSpanIsPositive', () => {
    const single: PoseGraphEdge[] = [
      { from: 1, to: 2, type: 'odometry', residual: 0.03 },
    ];
    const stats = computeResidualStats(single);
    expect(stats.odometry.max).toBeGreaterThan(stats.odometry.min);
  });
});

// --------------------------------------------------------- residualColor --

describe('residualColor', () => {
  it('LowResidual_ReturnsBaseColor', () => {
    const base = EDGE_BASE_COLORS.odometry;
    const [r, g, b] = residualColor(0, 0, 1, base);
    expect(r).toBeCloseTo(base[0]);
    expect(g).toBeCloseTo(base[1]);
    expect(b).toBeCloseTo(base[2]);
  });

  it('HighResidual_ReturnsHotRed', () => {
    const base = EDGE_BASE_COLORS.odometry;
    const [r, g, b] = residualColor(1, 0, 1, base);
    expect(r).toBeCloseTo(1.0);
    expect(g).toBeCloseTo(0.2);
    expect(b).toBeCloseTo(0.1);
  });

  it('MidResidual_InterpolatesLinearly', () => {
    const base: [number, number, number] = [0.0, 0.0, 0.0];
    const [r] = residualColor(0.5, 0, 1, base);
    // Midpoint between 0 (base) and 1.0 (hot red)
    expect(r).toBeCloseTo(0.5);
  });

  it('ZeroSpan_ReturnsBaseColor', () => {
    const base = EDGE_BASE_COLORS.loop_closure;
    // min === max → span 0, t clamped to 0 → base colour
    const [r, g, b] = residualColor(0.3, 0.3, 0.3, base);
    expect(r).toBeCloseTo(base[0]);
    expect(g).toBeCloseTo(base[1]);
    expect(b).toBeCloseTo(base[2]);
  });
});

// --------------------------------------------------------- posegraphNote --

describe('posegraphNote', () => {
  it('NullGraph_ReturnsNull', () => {
    expect(posegraphNote(null)).toBeNull();
  });

  it('UndefinedGraph_ReturnsNull', () => {
    expect(posegraphNote(undefined)).toBeNull();
  });

  it('Error_ReturnsErrorMessage', () => {
    const note = posegraphNote(null, new Error('network timeout'));
    expect(note).toContain('network timeout');
  });

  it('EmptyNodes_TellsUserToRunCreateClouds', () => {
    const note = posegraphNote({ nodes: [], edges: [] });
    expect(note).toContain('rux create clouds');
  });

  it('NodesButNoEdges_PromptOptimize', () => {
    const note = posegraphNote({ nodes: GRAPH.nodes, edges: [] });
    expect(note).toContain('rux optimize');
    expect(note).toContain('3'); // 3 frames
  });

  it('FullGraph_SummarisesFramesAndEdges', () => {
    const note = posegraphNote(GRAPH);
    expect(note).toContain('3'); // frames
    expect(note).toContain('2'); // 2 odometry edges
    expect(note).toContain('2'); // 2 loop/panorama edges counted as "loop"
  });
});

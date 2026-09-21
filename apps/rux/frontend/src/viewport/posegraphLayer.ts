// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pose-graph layer helpers that do not need a GPU or a WebGL context (#265).
 *
 * Kept apart from `PoseGraphScene.ts` for the same reason `gsplatLayer.ts` is
 * kept apart from `SplatScene.ts`: the colour-mapping and description logic is
 * plain arithmetic and is unit-testable without a renderer.
 */

import type { PoseGraph, PoseGraphEdge, PoseGraphEdgeType } from '../api/types';

// ---------------------------------------------------------- edge colours ----

/**
 * Base RGB colour (0–1 range) for each edge type.
 *
 * Odometry edges form the backbone — kept visually quiet.
 * Loop-closure edges carry the correction signal — warm colour family.
 * Panorama-derived loop edges are distinguishable from frame-to-frame loops.
 */
export const EDGE_BASE_COLORS: Record<PoseGraphEdgeType, [number, number, number]> =
  {
    odometry: [0.35, 0.35, 0.45],
    loop_closure: [0.2, 0.5, 1.0],
    panorama: [0.7, 0.3, 0.9],
  };

/**
 * Map a post-solve residual value to an RGB heat colour (0–1 range).
 *
 * Low residual → base colour (satisfied constraint).
 * High residual → red (strained constraint).
 *
 * @param residual    The GTSAM per-factor cost (0.5 × whitened squared residual).
 * @param minResidual Minimum residual in the dataset (maps to t=0).
 * @param maxResidual Maximum residual in the dataset (maps to t=1).
 * @param baseColor   The type's base colour, used when t=0.
 */
export function residualColor(
  residual: number,
  minResidual: number,
  maxResidual: number,
  baseColor: [number, number, number],
): [number, number, number] {
  const span = maxResidual - minResidual;
  const t = span > 0 ? Math.max(0, Math.min(1, (residual - minResidual) / span)) : 0;
  // Interpolate base colour → [1, 0.2, 0.1] (vivid red) linearly.
  const hot: [number, number, number] = [1.0, 0.2, 0.1];
  return [
    baseColor[0] + t * (hot[0] - baseColor[0]),
    baseColor[1] + t * (hot[1] - baseColor[1]),
    baseColor[2] + t * (hot[2] - baseColor[2]),
  ];
}

// --------------------------------------------------- residual statistics ----

/** Min/max residual split by edge type, used to normalise the heat map. */
export interface ResidualStats {
  min: number;
  max: number;
}

/**
 * Compute per-type residual statistics from the full edge list.
 *
 * Separate statistics per type keep the odometry heat map from washing out the
 * loop-closure one — the two populations have very different residual scales.
 */
export function computeResidualStats(
  edges: PoseGraphEdge[],
): Record<PoseGraphEdgeType, ResidualStats> {
  const init = (): ResidualStats => ({ min: Infinity, max: -Infinity });
  const stats: Record<PoseGraphEdgeType, ResidualStats> = {
    odometry: init(),
    loop_closure: init(),
    panorama: init(),
  };
  for (const e of edges) {
    const s = stats[e.type];
    if (e.residual < s.min) s.min = e.residual;
    if (e.residual > s.max) s.max = e.residual;
  }
  // Guard: an all-same residual (or a single edge) produces min === max, which
  // would make the heat map all-zero.  Expand to a minimal span around the value.
  for (const key of Object.keys(stats) as PoseGraphEdgeType[]) {
    const s = stats[key];
    if (!isFinite(s.min)) {
      s.min = 0;
      s.max = 1;
    } else if (s.min === s.max) {
      s.min = Math.max(0, s.min - 0.001);
      s.max = s.max + 0.001;
    }
  }
  return stats;
}

// --------------------------------------------------------- descriptions ----

const COUNT = new Intl.NumberFormat();

/** One-line summary for the layer-panel row. */
export function posegraphNote(graph: PoseGraph | null | undefined, error?: Error | null): string | null {
  if (error) return `Could not load pose graph: ${error.message}`;
  if (!graph) return null;
  if (graph.nodes.length === 0) return 'No poses stored. Run `rux create clouds` first.';
  if (graph.edges.length === 0)
    return (
      `${COUNT.format(graph.nodes.length)} frames — run ` +
      '`rux optimize` to store edges.'
    );
  const odom = graph.edges.filter((e) => e.type === 'odometry').length;
  const loops = graph.edges.filter((e) => e.type !== 'odometry').length;
  const parts = [`${COUNT.format(graph.nodes.length)} frames`, `${COUNT.format(odom)} odometry`];
  if (loops > 0) parts.push(`${COUNT.format(loops)} loop`);
  return parts.join(' · ');
}

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

// --------------------------------------------------- projection plane (#445) ---

/**
 * Which world-space plane to flatten the graph onto for 2D inspection.
 * '3D' means no flattening (the default orbit view).
 */
export type ProjectionPlane = '3D' | 'XY' | 'XZ' | 'YZ';

/**
 * Project a world-space position onto the chosen plane by zeroing the
 * out-of-plane axis.  Returns a [x, y, z] triple — kept as plain numbers so
 * this helper is testable without a WebGL context.
 */
export function projectPosition(
  x: number,
  y: number,
  z: number,
  plane: ProjectionPlane,
): [number, number, number] {
  switch (plane) {
    case 'XY': return [x, y, 0];
    case 'XZ': return [x, 0, z];
    case 'YZ': return [0, y, z];
    default:   return [x, y, z];
  }
}

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
 * Colour used to highlight edges whose residual exceeds the user-set
 * threshold (#445).  Vivid amber — distinct from both the type colours and
 * the red heat ramp so it reads as a manual selection, not a heatmap level.
 */
export const THRESHOLD_COLOR: [number, number, number] = [1.0, 0.75, 0.0];

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

// -------------------------------------------------- node degree helpers (#445) -

/**
 * Compute the degree (edge count) of every node from a full graph.
 *
 * Edges that reference a node id not in `graph.nodes` are ignored — the
 * same guard `PoseGraphScene.buildEdges` applies to positions.
 */
export function computeNodeDegrees(graph: PoseGraph): Map<number, number> {
  const nodeIds = new Set(graph.nodes.map((n) => n.id));
  const degrees = new Map<number, number>();
  for (const n of graph.nodes) degrees.set(n.id, 0);
  for (const e of graph.edges) {
    if (!nodeIds.has(e.from) || !nodeIds.has(e.to)) continue;
    degrees.set(e.from, (degrees.get(e.from) ?? 0) + 1);
    degrees.set(e.to, (degrees.get(e.to) ?? 0) + 1);
  }
  return degrees;
}

/**
 * Map a node's degree to an RGB colour (0–1 range) for the degree-overlay mode.
 *
 * Low degree (sparse, weakly constrained) → blue.
 * High degree (hub, over-determined) → red.
 *
 * @param degree    The node's edge count.
 * @param maxDegree Maximum degree in the graph — defines the top of the ramp.
 *                  Must be ≥ 1; pass 1 for a single-node graph.
 */
export function nodeDegreeColor(
  degree: number,
  maxDegree: number,
): [number, number, number] {
  if (maxDegree <= 0) return [0.85, 0.85, 0.9];
  const t = Math.min(1, degree / maxDegree);
  const low: [number, number, number]  = [0.2, 0.4, 0.9];
  const high: [number, number, number] = [1.0, 0.2, 0.1];
  return [
    low[0] + t * (high[0] - low[0]),
    low[1] + t * (high[1] - low[1]),
    low[2] + t * (high[2] - low[2]),
  ];
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

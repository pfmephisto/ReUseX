// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Three.js scene for the pose-graph layer (#265, review point 4; #445).
 *
 * Nodes: `THREE.Points` — one per sensor frame, vertex-coloured either as a
 *        uniform grey or by connectivity degree.
 * Edges: one `THREE.LineSegments` per edge type, so each type can be shown
 *        or hidden independently without rebuilding geometry.  Within each
 *        segment the vertex colour encodes the residual heat ramp; edges above
 *        the user-set threshold are amber instead.
 *
 * Projection: positions are flattened to the chosen plane (XY/XZ/YZ) before
 *        upload so the graph can be inspected in 2D alongside the matching
 *        camera preset — no GPU shader needed.
 */

import * as THREE from 'three';
import type { PoseGraph, PoseGraphEdge, PoseGraphEdgeType } from '../api/types';
import {
  EDGE_BASE_COLORS,
  THRESHOLD_COLOR,
  computeNodeDegrees,
  computeResidualStats,
  nodeDegreeColor,
  projectPosition,
  residualColor,
  type ProjectionPlane,
  type ResidualStats,
} from './posegraphLayer';

/** Per-type edge batch stored for in-place recolouring. */
interface EdgeBatch {
  lines: THREE.LineSegments;
  edges: PoseGraphEdge[];
  stats: ResidualStats;
}

/** Extract world-space position from a column-major 4×4 pose, projected. */
function posFromPose(pose: number[], plane: ProjectionPlane): THREE.Vector3 {
  const [x, y, z] = projectPosition(pose[12], pose[13], pose[14], plane);
  return new THREE.Vector3(x, y, z);
}

export class PoseGraphScene {
  private readonly scene: THREE.Scene;
  private nodePoints: THREE.Points | null = null;
  private edgesByType: Map<PoseGraphEdgeType, EdgeBatch> = new Map();

  // Persistent state — survives clear/rebuild cycles.
  private visible_ = true;
  private graph_: PoseGraph | null = null;
  private projectionPlane_: ProjectionPlane = '3D';
  private edgeTypeVisible_: Record<PoseGraphEdgeType, boolean> = {
    odometry: true,
    loop_closure: true,
    panorama: true,
  };
  private residualThreshold_ = 0;
  private nodeColorMode_: 'default' | 'degree' = 'default';

  constructor(scene: THREE.Scene) {
    this.scene = scene;
  }

  /** Load or replace the pose graph. Call with null to clear. */
  load(graph: PoseGraph | null): void {
    this.graph_ = graph;
    this.rebuild();
  }

  /** Switch the 2D projection plane.  Rebuilds geometry; keeps all other settings. */
  setProjectionPlane(plane: ProjectionPlane): void {
    this.projectionPlane_ = plane;
    this.rebuild();
  }

  /** Show or hide one edge type without rebuilding geometry. */
  setEdgeTypeVisible(type: PoseGraphEdgeType, visible: boolean): void {
    this.edgeTypeVisible_[type] = visible;
    const batch = this.edgesByType.get(type);
    if (batch) batch.lines.visible = this.visible_ && visible;
  }

  /**
   * Highlight edges above `threshold` in amber; 0 disables the threshold.
   * Updates vertex colour buffers in-place — no geometry rebuild.
   */
  setResidualThreshold(threshold: number): void {
    this.residualThreshold_ = threshold;
    this.recolorEdges();
  }

  /** Switch node colouring between uniform grey and per-node connectivity degree. */
  setNodeColorMode(mode: 'default' | 'degree'): void {
    this.nodeColorMode_ = mode;
    this.recolorNodes();
  }

  setVisible(visible: boolean): void {
    this.visible_ = visible;
    if (this.nodePoints) this.nodePoints.visible = visible;
    for (const [type, { lines }] of this.edgesByType) {
      lines.visible = visible && (this.edgeTypeVisible_[type] ?? true);
    }
  }

  isVisible(): boolean {
    return this.visible_;
  }

  dispose(): void {
    this.clear();
  }

  // ---------------------------------------------------------------- private --

  private rebuild(): void {
    this.clear();
    const graph = this.graph_;
    if (!graph || graph.nodes.length === 0) return;

    const posMap = new Map<number, THREE.Vector3>();
    for (const n of graph.nodes)
      posMap.set(n.id, posFromPose(n.pose, this.projectionPlane_));

    this.buildNodes(graph, posMap);
    this.buildEdges(graph, posMap);
  }

  private buildNodes(graph: PoseGraph, posMap: Map<number, THREE.Vector3>): void {
    const count = graph.nodes.length;
    const positions = new Float32Array(count * 3);
    let i = 0;
    for (const n of graph.nodes) {
      const p = posMap.get(n.id)!;
      positions[i++] = p.x;
      positions[i++] = p.y;
      positions[i++] = p.z;
    }

    const nodeColors = this.computeNodeColors(graph);

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(nodeColors, 3));

    const mat = new THREE.PointsMaterial({
      size: 0.04,
      vertexColors: true,
      sizeAttenuation: true,
    });

    this.nodePoints = new THREE.Points(geo, mat);
    this.nodePoints.visible = this.visible_;
    this.scene.add(this.nodePoints);
  }

  private computeNodeColors(graph: PoseGraph): Float32Array {
    const colors = new Float32Array(graph.nodes.length * 3);
    if (this.nodeColorMode_ === 'degree') {
      const degrees = computeNodeDegrees(graph);
      const maxDegree = Math.max(1, ...degrees.values());
      let i = 0;
      for (const n of graph.nodes) {
        const [r, g, b] = nodeDegreeColor(degrees.get(n.id) ?? 0, maxDegree);
        colors[i++] = r;
        colors[i++] = g;
        colors[i++] = b;
      }
    } else {
      for (let i = 0; i < graph.nodes.length; i++) {
        colors[i * 3 + 0] = 0.85;
        colors[i * 3 + 1] = 0.85;
        colors[i * 3 + 2] = 0.90;
      }
    }
    return colors;
  }

  private recolorNodes(): void {
    if (!this.nodePoints || !this.graph_) return;
    const colors = this.computeNodeColors(this.graph_);
    const attr = this.nodePoints.geometry.getAttribute('color') as THREE.BufferAttribute;
    (attr.array as Float32Array).set(colors);
    attr.needsUpdate = true;
  }

  private buildEdges(graph: PoseGraph, posMap: Map<number, THREE.Vector3>): void {
    const validEdges = graph.edges.filter(
      (e) => posMap.has(e.from) && posMap.has(e.to),
    );
    if (validEdges.length === 0) return;

    const stats = computeResidualStats(validEdges);

    // Group by type so each gets its own LineSegments (independent visibility).
    const byType = new Map<PoseGraphEdgeType, PoseGraphEdge[]>();
    for (const e of validEdges) {
      if (!byType.has(e.type)) byType.set(e.type, []);
      byType.get(e.type)!.push(e);
    }

    for (const [type, edges] of byType) {
      const positions = new Float32Array(edges.length * 6); // 2 endpoints × xyz
      const colors = new Float32Array(edges.length * 6);    // 2 vertices × rgb
      const base = EDGE_BASE_COLORS[type] ?? ([0.5, 0.5, 0.5] as [number, number, number]);
      const s = stats[type];

      for (let i = 0; i < edges.length; i++) {
        const e = edges[i];
        const from = posMap.get(e.from)!;
        const to = posMap.get(e.to)!;

        const aboveThreshold = this.residualThreshold_ > 0 && e.residual > this.residualThreshold_;
        const [r, g, b] = aboveThreshold
          ? THRESHOLD_COLOR
          : residualColor(e.residual, s.min, s.max, base);

        const pi = i * 6;
        positions[pi + 0] = from.x;
        positions[pi + 1] = from.y;
        positions[pi + 2] = from.z;
        positions[pi + 3] = to.x;
        positions[pi + 4] = to.y;
        positions[pi + 5] = to.z;

        colors[pi + 0] = r; colors[pi + 1] = g; colors[pi + 2] = b;
        colors[pi + 3] = r; colors[pi + 4] = g; colors[pi + 5] = b;
      }

      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

      const mat = new THREE.LineBasicMaterial({
        vertexColors: true,
        linewidth: 1,
      });

      const lines = new THREE.LineSegments(geo, mat);
      lines.visible = this.visible_ && (this.edgeTypeVisible_[type] ?? true);
      this.scene.add(lines);

      this.edgesByType.set(type, { lines, edges, stats: s });
    }
  }

  /**
   * Recolour edges in-place after a threshold change.
   * Only the colour buffer changes — positions are left untouched.
   */
  private recolorEdges(): void {
    for (const [type, { lines, edges, stats }] of this.edgesByType) {
      const base = EDGE_BASE_COLORS[type] ?? ([0.5, 0.5, 0.5] as [number, number, number]);
      const colorAttr = lines.geometry.getAttribute('color') as THREE.BufferAttribute;
      const colors = colorAttr.array as Float32Array;

      for (let i = 0; i < edges.length; i++) {
        const e = edges[i];
        const aboveThreshold = this.residualThreshold_ > 0 && e.residual > this.residualThreshold_;
        const [r, g, b] = aboveThreshold
          ? THRESHOLD_COLOR
          : residualColor(e.residual, stats.min, stats.max, base);

        colors[i * 6 + 0] = r; colors[i * 6 + 1] = g; colors[i * 6 + 2] = b;
        colors[i * 6 + 3] = r; colors[i * 6 + 4] = g; colors[i * 6 + 5] = b;
      }
      colorAttr.needsUpdate = true;
    }
  }

  private clear(): void {
    if (this.nodePoints) {
      this.nodePoints.geometry.dispose();
      (this.nodePoints.material as THREE.Material).dispose();
      this.scene.remove(this.nodePoints);
      this.nodePoints = null;
    }
    for (const { lines } of this.edgesByType.values()) {
      lines.geometry.dispose();
      (lines.material as THREE.Material).dispose();
      this.scene.remove(lines);
    }
    this.edgesByType.clear();
  }
}

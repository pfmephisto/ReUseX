// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Three.js scene for the pose-graph layer (#265, review point 4).
 *
 * Nodes: `THREE.Points` — one sphere-ish point per sensor frame with a pose.
 * Edges: `THREE.LineSegments` — one line segment per constraint, vertex-coloured
 *        by edge type with residual mapped to a heat ramp (low = type colour,
 *        high = red).
 *
 * Kept out of any React component so it can be unit-tested without jsdom and
 * to mirror the `MeshScene` / `SplatScene` / `PanoramaScene` pattern.
 */

import * as THREE from 'three';
import type { PoseGraph } from '../api/types';
import type { PoseGraphEdge } from '../api/types';
import {
  EDGE_BASE_COLORS,
  computeResidualStats,
  residualColor,
} from './posegraphLayer';

/** Extract world-space position from a column-major 4×4 pose. */
function positionFromPose(pose: number[]): THREE.Vector3 {
  return new THREE.Vector3(pose[12], pose[13], pose[14]);
}

export class PoseGraphScene {
  private readonly scene: THREE.Scene;
  private nodePoints: THREE.Points | null = null;
  private edgeLines: THREE.LineSegments | null = null;
  private visible_ = true;

  constructor(scene: THREE.Scene) {
    this.scene = scene;
  }

  /** Load or replace the pose graph. Call with null to clear. */
  load(graph: PoseGraph | null): void {
    this.clear();
    if (!graph || graph.nodes.length === 0) return;

    // Build a node-id → position map.
    const posMap = new Map<number, THREE.Vector3>();
    for (const n of graph.nodes)
      posMap.set(n.id, positionFromPose(n.pose));

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

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.PointsMaterial({
      size: 0.04,
      color: new THREE.Color(0.85, 0.85, 0.9),
      sizeAttenuation: true,
    });

    this.nodePoints = new THREE.Points(geo, mat);
    this.nodePoints.visible = this.visible_;
    this.scene.add(this.nodePoints);
  }

  private buildEdges(graph: PoseGraph, posMap: Map<number, THREE.Vector3>): void {
    // Only build edges that have both endpoints in the pose map.
    const validEdges: PoseGraphEdge[] = graph.edges.filter(
      (e) => posMap.has(e.from) && posMap.has(e.to),
    );
    if (validEdges.length === 0) return;

    const stats = computeResidualStats(validEdges);

    const positions = new Float32Array(validEdges.length * 6); // 2 endpoints × xyz
    const colors = new Float32Array(validEdges.length * 6);    // 2 vertices × rgb

    for (let i = 0; i < validEdges.length; i++) {
      const e = validEdges[i];
      const from = posMap.get(e.from)!;
      const to = posMap.get(e.to)!;

      const base = EDGE_BASE_COLORS[e.type] ?? ([0.5, 0.5, 0.5] as [number, number, number]);
      const s = stats[e.type];
      const [r, g, b] = residualColor(e.residual, s.min, s.max, base);

      const pi = i * 6;
      positions[pi + 0] = from.x;
      positions[pi + 1] = from.y;
      positions[pi + 2] = from.z;
      positions[pi + 3] = to.x;
      positions[pi + 4] = to.y;
      positions[pi + 5] = to.z;

      colors[pi + 0] = r;
      colors[pi + 1] = g;
      colors[pi + 2] = b;
      colors[pi + 3] = r;
      colors[pi + 4] = g;
      colors[pi + 5] = b;
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

    const mat = new THREE.LineBasicMaterial({
      vertexColors: true,
      linewidth: 1, // >1 only works on some platforms; 1 is universal
    });

    this.edgeLines = new THREE.LineSegments(geo, mat);
    this.edgeLines.visible = this.visible_;
    this.scene.add(this.edgeLines);
  }

  setVisible(visible: boolean): void {
    this.visible_ = visible;
    if (this.nodePoints) this.nodePoints.visible = visible;
    if (this.edgeLines) this.edgeLines.visible = visible;
  }

  isVisible(): boolean {
    return this.visible_;
  }

  dispose(): void {
    this.clear();
  }

  private clear(): void {
    if (this.nodePoints) {
      this.nodePoints.geometry.dispose();
      (this.nodePoints.material as THREE.Material).dispose();
      this.scene.remove(this.nodePoints);
      this.nodePoints = null;
    }
    if (this.edgeLines) {
      this.edgeLines.geometry.dispose();
      (this.edgeLines.material as THREE.Material).dispose();
      this.scene.remove(this.edgeLines);
      this.edgeLines = null;
    }
  }
}

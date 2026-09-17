# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Read frame positions and thumbnails from a .rux project database.

This module contains only database-reading logic; it imports nothing from the
loop_edges package so it can be imported in the Gradio venv without issue.
"""

import json
import sqlite3
import struct
from pathlib import Path

import cv2
import numpy as np


def decode_pose(blob: bytes) -> np.ndarray:
    """Decode a 16×float64 row-major blob to a 4×4 numpy matrix."""
    return np.array(struct.unpack("16d", blob)).reshape(4, 4)


def read_seed_positions(db_path: str) -> list[tuple[int, np.ndarray | None]]:
    """Return [(node_id, xyz_or_None)] for all sensor frames with a pose.

    xyz is the camera world position (last column, first 3 rows of the
    world-from-camera transform blob).
    """
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute(
        "SELECT node_id, transform FROM sensor_frames "
        "WHERE color IS NOT NULL AND depth IS NOT NULL ORDER BY node_id"
    )
    rows = cur.fetchall()
    con.close()

    result = []
    for node_id, blob in rows:
        if blob is None or len(blob) < 128:
            result.append((node_id, None))
            continue
        mat = decode_pose(blob)
        xyz = mat[:3, 3].copy()
        result.append((node_id, xyz))
    return result


def read_color_thumbnail(db_path: str, node_id: int, max_size: int = 320) -> np.ndarray | None:
    """Return an RGB thumbnail for a sensor frame, or None if not found."""
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute("SELECT color FROM sensor_frames WHERE node_id=?", (node_id,))
    row = cur.fetchone()
    con.close()
    if row is None or row[0] is None:
        return None
    img = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    scale = max_size / max(h, w)
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def read_depth_thumbnail(db_path: str, node_id: int, max_size: int = 320) -> np.ndarray | None:
    """Return a colourized depth thumbnail (RGB), or None if not found."""
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute("SELECT depth FROM sensor_frames WHERE node_id=?", (node_id,))
    row = cur.fetchone()
    con.close()
    if row is None or row[0] is None:
        return None
    depth = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None:
        return None
    # Normalize to 0-255 for display; 0 = invalid → black.
    valid = depth > 0
    if not valid.any():
        return None
    mn, mx = depth[valid].min(), depth[valid].max()
    norm = np.zeros_like(depth, dtype=np.uint8)
    if mx > mn:
        norm[valid] = ((depth[valid] - mn) / (mx - mn) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    h, w = colored.shape[:2]
    scale = max_size / max(h, w)
    if scale < 1.0:
        colored = cv2.resize(colored, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)


def read_all_poses(db_path: str) -> dict[int, np.ndarray]:
    """Return {node_id: 4x4 transform} for all frames with a pose blob."""
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id")
    rows = cur.fetchall()
    con.close()
    poses = {}
    for nid, blob in rows:
        if blob and len(blob) == 128:
            poses[nid] = decode_pose(blob)
    return poses


def read_frame_count(db_path: str) -> int:
    """Return total number of sensor frames with color+depth."""
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute(
        "SELECT COUNT(*) FROM sensor_frames WHERE color IS NOT NULL AND depth IS NOT NULL"
    )
    count = cur.fetchone()[0]
    con.close()
    return count


def propose_candidates(
    positions: list[tuple[int, np.ndarray | None]],
    query_node_id: int,
    n_candidates: int = 20,
    min_frame_gap: int = 50,
) -> list[tuple[int, float]]:
    """Return [(node_id, distance_m)] of the closest frames by seed-pose proximity.

    Excludes frames within min_frame_gap of the query frame to avoid
    near-consecutive frames (which are trivially aligned).
    """
    # Build index map: node_id -> list index
    id_to_idx = {nid: i for i, (nid, _) in enumerate(positions)}
    query_idx = id_to_idx.get(query_node_id)
    query_xyz = None
    for nid, xyz in positions:
        if nid == query_node_id:
            query_xyz = xyz
            break

    if query_xyz is None or query_idx is None:
        return []

    candidates = []
    for i, (nid, xyz) in enumerate(positions):
        if xyz is None:
            continue
        if abs(i - query_idx) < min_frame_gap:
            continue
        dist = float(np.linalg.norm(xyz - query_xyz))
        candidates.append((nid, dist))

    candidates.sort(key=lambda x: x[1])
    return candidates[:n_candidates]

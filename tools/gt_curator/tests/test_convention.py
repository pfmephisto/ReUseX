# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Unit tests for the T_ij convention and OpenCV match→T_ij pipeline.

Tests are PURE logic — they do not touch a real .rux database.
They verify:
  1. T_ij convention: on a synthetic pair with a known rigid transform,
     ransac_pose recovers T_ij s.t. p_i ≈ T_ij @ p_j (frame-j → frame-i).
  2. The T_ij from a near-consecutive pair is consistent with the seed-pose
     relative: T_ij ≈ pose(i)^-1 @ pose(j).
  3. Anchor export/import round-trips correctly.
  4. ICP reduces RMS on a synthetic offset.
"""

import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

# Make the test importable from the project root as well.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "loop_edges"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from export_loop_edges import backproject, ransac_pose  # noqa: E402
from anchors import AnchorList  # noqa: E402
from icp_align import icp, sliders_to_T  # noqa: E402


RNG_SEED = 42


# --------------------------------------------------------------------------- #
# Helpers                                                                       #
# --------------------------------------------------------------------------- #

def _make_frame(n=200, noise=0.005, depth_val=2.0):
    """Synthetic frame: Nx2 pixel coords, Nx3 3D points, 3x3 K."""
    rng = np.random.default_rng(RNG_SEED)
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
    # Random valid pixel coordinates
    u = rng.uniform(10, 630, n)
    v = rng.uniform(10, 470, n)
    xy = np.stack([u, v], axis=1)
    z = np.full(n, depth_val) + rng.normal(0, 0.02, n)
    z = np.clip(z, 0.3, 5.0)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    pts3 = np.stack([x, y, z], axis=1)
    return xy, pts3, K


def _apply_T(pts, T):
    """Apply 4x4 T to Nx3 points."""
    return (pts @ T[:3, :3].T) + T[:3, 3]


def _make_T(tx=0.2, ty=0.1, tz=0.05, angle_deg=5.0):
    """Build a known rigid transform for testing."""
    R = Rotation.from_euler("z", angle_deg, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


# --------------------------------------------------------------------------- #
# Test 1: T_ij convention (pure geometry, no DB)                               #
# --------------------------------------------------------------------------- #

class TestTijConvention:
    """T_ij = pose(i)^-1 @ pose(j) in world-from-camera convention.

    Equivalently: p_i ≈ T_ij @ p_j  (maps point from j's optical frame to i's).
    """

    def test_ransac_pose_recovers_known_T(self):
        """ransac_pose(src=p_j, dst=p_i) → T s.t. p_i ≈ T @ p_j."""
        rng = np.random.default_rng(RNG_SEED)
        _, p_i, _ = _make_frame(n=300)
        T_true = _make_T(tx=0.15, ty=0.08, tz=0.03, angle_deg=4.0)
        # p_j in j's optical frame; T_true maps j → i
        p_j = _apply_T(p_i, np.linalg.inv(T_true))
        # Add small noise
        p_j += np.random.default_rng(RNG_SEED + 1).normal(0, 0.002, p_j.shape)

        T_est, inl = ransac_pose(p_j, p_i, thresh=0.05, iters=500, rng=rng)
        assert T_est is not None, "ransac_pose returned None"
        assert inl is not None
        assert int(inl.sum()) > 200, f"too few inliers: {inl.sum()}"

        # Translation error < 5 mm
        t_err = float(np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]))
        assert t_err < 0.005, f"translation error {t_err:.4f}m exceeds 5mm"

        # Rotation error < 0.5 deg
        dR = T_est[:3, :3] @ T_true[:3, :3].T
        angle_err = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
        assert angle_err < 0.5, f"rotation error {angle_err:.3f}° exceeds 0.5°"

    def test_T_ij_consistent_with_seed_relative(self):
        """T_ij ≈ pose(i)^-1 @ pose(j) on a synthetic 'consecutive' pair."""
        # Build two world-from-camera poses
        pose_i = np.eye(4)
        pose_i[:3, 3] = [1.0, 0.0, 0.0]
        pose_i[:3, :3] = Rotation.from_euler("z", 10, degrees=True).as_matrix()

        T_rel = _make_T(tx=0.05, ty=0.02, tz=0.01, angle_deg=1.0)
        pose_j = pose_i @ T_rel  # T_ij = pose(i)^-1 @ pose(j)
        T_ij_expected = np.linalg.inv(pose_i) @ pose_j

        # Build 3D correspondences in optical frames
        rng = np.random.default_rng(RNG_SEED)
        p_i_optical = rng.uniform(-1, 1, (200, 3)) + [0, 0, 2]
        p_j_optical = _apply_T(p_i_optical, np.linalg.inv(T_ij_expected))

        T_est, inl = ransac_pose(p_j_optical, p_i_optical, thresh=0.05, iters=500, rng=rng)
        assert T_est is not None

        # The estimated T_ij should match pose(i)^-1 @ pose(j)
        t_err = float(np.linalg.norm(T_est[:3, 3] - T_ij_expected[:3, 3]))
        assert t_err < 0.005, (
            f"T_ij translation error {t_err:.4f}m vs expected\n"
            f"  estimated: {T_est[:3,3]}\n  expected:  {T_ij_expected[:3,3]}"
        )

    def test_seed_relative_round_trip(self):
        """Verify the exact comparison the spec asks for.

        On a near-consecutive pair, the computed T_ij must ≈
        seed_pose(i)^-1 · seed_pose(j).  This is the critical convention check.
        """
        rng = np.random.default_rng(RNG_SEED + 99)

        # Simulated seed poses (world-from-camera)
        pose_i = np.eye(4)
        pose_i[:3, :3] = Rotation.from_euler("y", 15, degrees=True).as_matrix()
        pose_i[:3, 3] = [2.0, 1.0, 0.0]

        step = _make_T(tx=0.03, ty=0.01, tz=0.005, angle_deg=0.8)
        pose_j = pose_i @ step

        T_ij_seed = np.linalg.inv(pose_i) @ pose_j  # what solve_gt_poses uses

        # Generate clean 3D correspondences in optical frames
        n = 400
        p_i_opt = rng.uniform(-0.5, 0.5, (n, 3)) + [0, 0, 1.5]
        # T_ij maps optical-j → optical-i: p_i = T_ij @ p_j => p_j = T_ij^-1 @ p_i
        p_j_opt = _apply_T(p_i_opt, np.linalg.inv(T_ij_seed))

        T_computed, inl = ransac_pose(p_j_opt, p_i_opt, thresh=0.05, iters=500, rng=rng)
        assert T_computed is not None

        t_diff = np.linalg.norm(T_computed[:3, 3] - T_ij_seed[:3, 3])
        dR = T_computed[:3, :3] @ T_ij_seed[:3, :3].T
        r_diff_deg = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))

        print(f"\n  seed T_ij translation: {T_ij_seed[:3,3]}")
        print(f"  computed T_ij:         {T_computed[:3,3]}")
        print(f"  translation diff:      {t_diff*1000:.2f}mm")
        print(f"  rotation diff:         {r_diff_deg:.4f}°")

        assert t_diff < 0.003, f"convention mismatch: {t_diff*1000:.2f}mm translation error"
        assert r_diff_deg < 0.3, f"convention mismatch: {r_diff_deg:.4f}° rotation error"


# --------------------------------------------------------------------------- #
# Test 2: Anchor list round-trip                                                #
# --------------------------------------------------------------------------- #

class TestAnchorList:
    def test_add_and_export_round_trip(self):
        al = AnchorList()
        T = np.eye(4)
        T[:3, 3] = [0.1, 0.2, 0.3]
        al.add(node_i=10, node_j=200, T_ij=T, method="orb", rms=0.012, n_inliers=150)
        al.add(node_i=50, node_j=800, T_ij=T * 1.1, method="xfeat", rms=0.008, n_inliers=320)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            path = tf.name
        try:
            al.export_json(path)
            doc = json.loads(Path(path).read_text())
            assert doc["schema"] == "reusex.gt_anchors.v1"
            assert len(doc["edges"]) == 2
            e0 = doc["edges"][0]
            assert e0["node_i"] == 10
            assert e0["node_j"] == 200
            assert e0["method"] == "orb"
            assert len(e0["T_ij"]) == 16
            assert abs(e0["T_ij"][3] - 0.1) < 1e-9  # T[0,3] = tx

            # Round-trip: import back
            al2 = AnchorList()
            n = al2.import_json(path)
            assert n == 2
            assert al2._edges[1]["method"] == "xfeat"
        finally:
            Path(path).unlink(missing_ok=True)

    def test_remove(self):
        al = AnchorList()
        T = np.eye(4)
        al.add(0, 100, T, "orb", 0.01)
        al.add(1, 200, T, "sift", 0.02)
        al.remove(0)
        assert len(al) == 1
        assert al._edges[0]["node_i"] == 1

    def test_display_rows(self):
        al = AnchorList()
        T = np.eye(4)
        al.add(5, 50, T, "akaze", 0.015, n_inliers=80)
        rows = al.to_display_rows()
        assert len(rows) == 1
        assert rows[0][1] == 5   # node_i
        assert rows[0][2] == 50  # node_j
        assert rows[0][3] == "akaze"

    def test_schema_matches_solve_convention(self):
        """Verify T_ij in exported JSON matches pose(i)^-1 @ pose(j)."""
        pose_i = np.eye(4)
        pose_i[:3, 3] = [1.0, 0.0, 0.0]
        pose_j = pose_i.copy()
        pose_j[:3, 3] = [1.1, 0.05, 0.0]
        T_ij = np.linalg.inv(pose_i) @ pose_j

        al = AnchorList()
        al.add(node_i=0, node_j=10, T_ij=T_ij, method="orb", rms=0.005, n_inliers=200)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            path = tf.name
        try:
            al.export_json(path)
            doc = json.loads(Path(path).read_text())
            T_loaded = np.array(doc["edges"][0]["T_ij"]).reshape(4, 4)
            T_expected = np.linalg.inv(pose_i) @ pose_j
            assert np.allclose(T_loaded, T_expected, atol=1e-10), (
                f"Exported T_ij doesn't match pose(i)^-1 @ pose(j)\n"
                f"loaded:\n{T_loaded}\nexpected:\n{T_expected}"
            )
        finally:
            Path(path).unlink(missing_ok=True)


# --------------------------------------------------------------------------- #
# Test 3: ICP reduces RMS on synthetic offset                                  #
# --------------------------------------------------------------------------- #

class TestICP:
    def _make_overlapping_cloud(self, n=800, seed=0):
        """Generate a cloud that resembles a room corner — dense enough that
        a ~15 cm offset still leaves substantial overlap for ICP to grip."""
        rng = np.random.default_rng(seed)
        # Floor plane at z=0
        floor = rng.uniform(-1.5, 1.5, (n // 3, 2))
        floor_pts = np.column_stack([floor, np.zeros(n // 3)])
        # Wall planes
        wall1 = rng.uniform(-1.5, 1.5, (n // 3, 2))
        wall1_pts = np.column_stack([wall1[:, 0], np.full(n // 3, -1.5), wall1[:, 1]])
        wall2 = rng.uniform(-1.5, 1.5, (n // 3, 2))
        wall2_pts = np.column_stack([np.full(n // 3, -1.5), wall2[:, 0], wall2[:, 1]])
        return np.vstack([floor_pts, wall1_pts, wall2_pts])

    def test_icp_reduces_rms_on_offset(self):
        """ICP with a small (5cm) offset should converge cleanly."""
        cloud = self._make_overlapping_cloud(n=900)
        # Use a small offset that ICP can handle without a good init
        T_true = sliders_to_T(0.05, -0.03, 0.02, 2.0)
        cloud_src = _apply_T(cloud, np.linalg.inv(T_true))

        from scipy.spatial import cKDTree
        tree = cKDTree(cloud)
        before_dists, _ = tree.query(cloud_src, workers=-1)
        rms_before = float(np.sqrt(np.mean(before_dists ** 2)))

        T_est, rms_after, overlap = icp(
            cloud_src, cloud, T_init=np.eye(4), max_iter=100,
            max_corr_m=0.3, trim_frac=0.90
        )

        print(f"\n  RMS before ICP: {rms_before*1000:.1f}mm")
        print(f"  RMS after ICP:  {rms_after*1000:.1f}mm")
        print(f"  Overlap:        {overlap*100:.1f}%")
        print(f"  T_est translation: {T_est[:3,3]}")
        print(f"  T_true translation: {T_true[:3,3]}")

        assert overlap > 0.0, "ICP found zero overlap — cloud setup problem"
        assert rms_after < rms_before, "ICP did not reduce RMS"
        assert rms_after < 0.05, f"ICP converged poorly: RMS={rms_after*1000:.1f}mm"

    def test_icp_with_correct_init_converges_tight(self):
        """ICP with the true T_init converges to near-zero RMS."""
        cloud = self._make_overlapping_cloud(n=600)
        T_true = sliders_to_T(0.10, 0.05, 0.0, 3.0)
        cloud_src = _apply_T(cloud, np.linalg.inv(T_true))

        T_est, rms_after, overlap = icp(
            cloud_src, cloud, T_init=T_true, max_iter=50,
            max_corr_m=0.2, trim_frac=0.90
        )
        print(f"\n  RMS after ICP (true init): {rms_after*1000:.2f}mm")
        print(f"  Overlap: {overlap*100:.1f}%")
        assert overlap > 0.5, f"Low overlap with true init: {overlap*100:.1f}%"
        assert rms_after < 0.02, f"ICP with true init should be near-zero, got {rms_after*1000:.2f}mm"

    def test_sliders_to_T_identity(self):
        T = sliders_to_T(0, 0, 0, 0)
        assert np.allclose(T, np.eye(4), atol=1e-10)

    def test_sliders_to_T_yaw_90(self):
        T = sliders_to_T(0, 0, 0, 90)
        # 90-degree yaw: x-axis → y-axis
        v = T[:3, :3] @ np.array([1, 0, 0])
        assert np.allclose(v, [0, 1, 0], atol=1e-10), f"Unexpected: {v}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

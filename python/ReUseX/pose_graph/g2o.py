from typing import Union, Any, Self

import numpy as np
import numpy.typing as npt

import g2o
from g2o import VertexSE3, EdgeSE3
from g2o.g2opy import Isometry3d

from scipy.spatial.transform import Rotation as R

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self) -> Self:
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations: int =20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose: Isometry3d, fixed:bool=False) -> None:
        v_se3 = VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices: Union[int, VertexSE3], measurement, 
                 information: npt.NDArray[Any] = np.identity(6),
            robust_kernel=None) -> None:

        edge = EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, np.int64):
                v = self.vertex(v)
            elif isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()


def compute_relative_transform(pos1, quat1, pos2, quat2):
    """
    Compute the relative transformation between two poses.
    
    Args:
        pos1: np.array of shape (3,) representing the first position.
        quat1: np.array of shape (4,) representing the first quaternion (x, y, z, w).
        pos2: np.array of shape (3,) representing the second position.
        quat2: np.array of shape (4,) representing the second quaternion (x, y, z, w).
    
    Returns:
        g2o.Isometry3d: The relative transformation from pose1 to pose2.
    """
    # Convert to g2o Isometry3d
    T1 = g2o.Isometry3d(g2o.Quaternion(*quat1), pos1)
    T2 = g2o.Isometry3d(g2o.Quaternion(*quat2), pos2)
    
    # Compute relative transformation
    relative_transform = T1.inverse() * T2
    
    return relative_transform


def homography_to_isometry(mat):
    """
    Convert a 3x3 homography matrix to g2o.Isometry3d
    Assumption: Homography represents a rigid transformation.
    
    Args:
        mat (numpy.ndarray): 3x3 homography matrix
    
    Returns:
        g2o.Isometry3d: SE(3) transformation
    """
    # Ensure the matrix is 3x3
    assert mat.shape == (3, 3), "Homography matrix should be 3x3."

    # Extract rotation (upper-left 3x3)
    R_matrix = mat[:3, :3]

    # Assume the translation is in the last column
    t = mat[:3, 2]

    # Convert rotation matrix to quaternion

    # Convert rotation matrix to quaternion using SciPy
    quat = R.from_matrix(R_matrix).as_quat()  # Returns (x, y, z, w)

    # Create g2o Quaternion (expects w as the first element)
    q = g2o.Quaternion(quat[3], quat[0], quat[1], quat[2])  # (w, x, y, z)

    # Create and return g2o Isometry3d
    return g2o.Isometry3d(q, t)

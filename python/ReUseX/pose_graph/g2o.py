# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

from typing import Union, Any, Self

import numpy as np
import numpy.typing as npt

import g2o
from g2o import VertexSE3, EdgeSE3
from g2o.g2opy import Isometry3d

from scipy.spatial.transform import Rotation as R

class PoseGraphOptimization(g2o.SparseOptimizer):
    """Pose graph optimization using g2o with SE3 vertices.
    
    This class extends g2o.SparseOptimizer to provide a simplified interface
    for pose graph optimization using SE3 transformations.
    """
    
    def __init__(self) -> Self:
        """Initialize the pose graph optimizer with Levenberg-Marquardt solver."""
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations: int = 200):
        """Run the optimization algorithm.
        
        Args:
            max_iterations: Maximum number of optimization iterations. Defaults to 200.
        """
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose: Isometry3d, fixed: bool = False) -> None:
        """Add a vertex (pose) to the graph.
        
        Args:
            id: Unique identifier for the vertex.
            pose: SE3 pose as Isometry3d.
            fixed: Whether this vertex is fixed during optimization. Defaults to False.
        """
        v_se3 = VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices: Union[int, VertexSE3], measurement, 
                 information: npt.NDArray[Any] = np.identity(6),
            robust_kernel=None) -> None:
        """Add an edge (constraint) between two vertices.
        
        Args:
            vertices: Tuple of vertex IDs or VertexSE3 objects.
            measurement: Relative pose measurement between vertices.
            information: Information matrix (inverse covariance). Defaults to identity.
            robust_kernel: Optional robust kernel for outlier rejection.
        """

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
        """Get the estimated pose for a vertex.
        
        Args:
            id: Vertex identifier.
            
        Returns:
            Estimated pose as Isometry3d.
        """
        return self.vertex(id).estimate()


class BundleAdjustment(g2o.SparseOptimizer):
    """Bundle adjustment optimizer using g2o.
    
    This class provides bundle adjustment functionality for simultaneous
    optimization of camera poses and 3D point positions.
    """
    
    def __init__(self):
        """Initialize the bundle adjustment optimizer with Levenberg-Marquardt solver."""
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=10):
        """Run the optimization algorithm.
        
        Args:
            max_iterations: Maximum number of optimization iterations. Defaults to 10.
        """
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_pose(self, pose_id, pose, cam, fixed=False):
        """Add a camera pose to the optimization.
        
        Args:
            pose_id: Unique identifier for the pose.
            pose: Camera pose.
            cam: Camera intrinsic parameters.
            fixed: Whether this pose is fixed during optimization. Defaults to False.
        """
        sbacam = g2o.SBACam(pose.orientation(), pose.position())
        sbacam.set_cam(cam.fx, cam.fy, cam.cx, cam.cy, cam.baseline)

        v_se3 = g2o.VertexCam()
        v_se3.set_id(pose_id * 2)   # internal id
        v_se3.set_estimate(sbacam)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3) 

    def add_point(self, point_id, point, fixed=False, marginalized=True):
        """Add a 3D point to the optimization.
        
        Args:
            point_id: Unique identifier for the point.
            point: 3D point coordinates.
            fixed: Whether this point is fixed during optimization. Defaults to False.
            marginalized: Whether to marginalize this point. Defaults to True.
        """
        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id * 2 + 1)
        v_p.set_estimate(point)
        v_p.set_marginalized(marginalized)
        v_p.set_fixed(fixed)
        super().add_vertex(v_p)

    def add_edge(self, point_id, pose_id, 
            measurement,
            information=np.identity(2),
            robust_kernel=g2o.RobustKernelHuber(np.sqrt(5.991))):   # 95% CI
        """Add an observation edge between a 3D point and camera pose.
        
        Args:
            point_id: ID of the 3D point.
            pose_id: ID of the camera pose.
            measurement: 2D image projection of the point.
            information: Information matrix. Defaults to 2x2 identity.
            robust_kernel: Robust kernel for outlier rejection. Defaults to Huber kernel.
        """

        edge = g2o.EdgeProjectP2MC()
        edge.set_vertex(0, self.vertex(point_id * 2 + 1))
        edge.set_vertex(1, self.vertex(pose_id * 2))
        edge.set_measurement(measurement)   # projection
        edge.set_information(information)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        """Get the estimated camera pose.
        
        Args:
            id: Vertex identifier.
            
        Returns:
            Estimated camera pose.
        """
        return self.vertex(id).estimate()

    #def get_pose(self, pose_id):
    #    return self.vertex(pose_id * 2).estimate()

    def get_point(self, point_id):
        """Get the estimated 3D point position.
        
        Args:
            point_id: Point identifier.
            
        Returns:
            Estimated 3D point coordinates.
        """
        return self.vertex(point_id * 2 + 1).estimate()


def compute_relative_transform(pos1, quat1, pos2, quat2):
    """Compute the relative transformation between two poses.
    
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
    """Convert a 3x3 homography matrix to g2o.Isometry3d.
    
    Assumes the homography represents a rigid transformation.
    
    Args:
        mat: 3x3 homography matrix as numpy.ndarray.
    
    Returns:
        g2o.Isometry3d representing the SE(3) transformation.
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

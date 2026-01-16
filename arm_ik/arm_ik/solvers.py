import logging
import math
from collections.abc import Callable
from typing import List

import numpy as np
from scipy.linalg import expm

import arm_ik.model_constants as M
from arm_ik.angles_utility import (
    map_joint_angles_from_software_to_physical,
    normalize_angle,
    normalize_angles,
)

logging.basicConfig(
    level=logging.DEBUG,  # Minimum level of messages to capture
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",  # Log format
    datefmt="%Y-%m-%d %H:%M:%S",  # Date format
)
logger = logging.getLogger(__name__)


# Minimum radius (XY plane) that goal_position must satisfy.
MINIMUM_RADIUS = 0.3


# =============================================================================
# COST FUNCTIONS:
# It's possible to have multiple solutions for desired pose of end effector.
# Use cost function to decide.
# =============================================================================
def _motion_cost(q_sols: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """
    Motion cost per solution.
    Returns shape (N,) normalized to [0, 1].
    """
    dist = np.abs(normalize_angles(q_sols - q_current))  # (N, K)

    # Use max distance (across joints) as cost for each solution.
    max_dist = np.max(dist, axis=1)  # (N,)
    return max_dist / np.pi


def _margin_cost(q_sols: np.ndarray, q_limits: np.ndarray) -> np.ndarray:
    """
    Margin cost per solution (lower = away from limits)
    Returns shape (N,) normalized to [0, 1].
    """
    q_min, q_max = q_limits[0], q_limits[1]

    dist_to_min = np.abs(q_sols - q_min)
    dist_to_max = np.abs(q_sols - q_max)

    # Get margin (min distance from its limit) for each joint
    margins = np.minimum(dist_to_min, dist_to_max)  # (N, K)

    # Use minimum margin (across joints) as score for each solution.
    min_margins = np.min(margins, axis=1)  # (N,)
    normalized_min_margins = np.clip(min_margins / (np.pi / 6), 0, 1)

    # Invert the score to get a cost
    return 1.0 - normalized_min_margins


def cost_min_joint_motion(
    q_sols: np.ndarray,
    q_current: np.ndarray,
    q_limits: np.ndarray,
) -> int:
    """
    Returns the index of the solution that minimizes the maximum joint motion required.

    Assumes q_sols are all within q_limits.

    Args:
        q_sols: Shape (N, K) - N solutions, K joints
        q_current: Shape (K,) - current joint angles
        q_limits: Shape (2, K) - [lower_limits, upper_limits] (unused here)
    """
    return int(np.argmin(_motion_cost(q_sols, q_current)))


def cost_max_margin_from_limits(
    q_sols: np.ndarray,
    q_current: np.ndarray,
    q_limits: np.ndarray,
) -> int:
    """
    Returns the index of the solution that maximizes distance from joint limits.

    Assumes q_sols are all within q_limits.

    Args:
        q_sols: Shape (N, K) - N solutions, K joints
        q_current: Shape (K,) - current joint angles (unused here)
        q_limits: Shape (2, K) - [lower_limits, upper_limits]
    """
    return int(np.argmin(_margin_cost(q_sols, q_limits)))


def cost_mixed(
    q_sols: np.ndarray,
    q_current: np.ndarray,
    q_limits: np.ndarray,
    motion_weight: float = 0.8,
    margin_weight: float = 0.2,
) -> int:
    """
    Returns the index of the solution that minimizes the weighted cost of motion and
    margin. It prioritizes minimal joint motion until very close to limits.
    """
    cost = motion_weight * _motion_cost(q_sols, q_current) + margin_weight * _margin_cost(
        q_sols, q_limits
    )
    return int(np.argmin(cost))


# =============================================================================
# SOLVER
# =============================================================================

# Type alias for cost function
CostFn = Callable[[np.ndarray, np.ndarray, np.ndarray], int]


class ArmSolver:
    """
    Analytical IK/FK solver for 6-DOF arm.

    It automatically loads geometry and joint limits from `model_constants.py`.
    By default it uses a mixed objective cost function for choosing best solution when
    multiple exists for desired EE pose.
    """

    def __init__(self):
        self.cost_fn = cost_mixed

        self.joint_limits = np.array(
            [
                np.deg2rad(M.JOINTS_LOWER_LIMITS),
                np.deg2rad(M.JOINTS_UPPER_LIMITS),
            ]
        )

        # This initial transformation comes from model assumption of initial
        # configuration of arm. See display.launch.py
        self.initial_T = np.array(
            [
                [0, 0, -1, 0],
                [0, 1, 0, 0],
                [1, 0, 0, M.UPPER_ARM + M.FOREARM + M.TIP_OFFSET_FROM_WRIST_CENTER],
                [0, 0, 0, 1],
            ]
        )

    def set_cost_function(self, fn: CostFn) -> None:
        self.cost_fn = fn

    def ik_solve(
        self,
        goal_position: np.ndarray,
        goal_orientation: np.ndarray,
        curr_joint_angles: np.ndarray,
    ) -> np.ndarray | None:
        """
        Solve inverse kinematics for desired end-effector pose.

        Args:
             goal_position: Shape (3,) - desired EE position in base frame
             goal_orientation: Shape (3,3) - desired EE orientation (rotation matrix)
             curr_joint_angles: Shape (6,) - current joint angles in radians,
                                required for resolving singularities

        Details:
        For non-redundant 6R arm like ours, analytical IK involves these steps:
        1) Determining the required wrist position for the goal position:
           goal position = wrist position + R * tip_offset.
           where R is orientation of wrist_roll frame, expressed in base frame.

        2) Determine the first three joints (turret, shoulder, elbow) to move to our
           wrist center to that required wrist center position. This step is called
           the inverse position kinematics.

        3) Determine the last three joints (elbow roll, wrist_pitch and wrist_roll)
           to rotate the wrist so that it aligns with the goal orientation.
           This step is called the inverse orientation kinematics.
           When that happens, the tip position will also be aligned to the
           goal position automatically.

        See more details in Notion notes.

        Returns:
            Shape (6,) array of joint angles, or None if no valid solution
        """
        # Don't allow a goal_position within a cylinder of radius r centered at
        # base frame's origin
        goal_radius = np.sqrt(goal_position[0] ** 2 + goal_position[1] ** 2)
        if goal_radius <= MINIMUM_RADIUS:
            logger.error("Goal radius on XY plane too close to base frame.")
            return None

        # 1. Compute required wrist center position
        #    goal_position = wrist_position + R @ tip_offset
        ee_offset = np.array([M.TIP_OFFSET_FROM_WRIST_CENTER, 0, 0]).reshape(3, 1)
        wrist_position = goal_position.reshape(3, 1) - goal_orientation @ ee_offset

        # Validate wrist is reachable
        max_wrist_distance = M.UPPER_ARM + M.FOREARM
        if np.linalg.norm(wrist_position) > max_wrist_distance:
            logger.error("Goal is out of range for the arm.")
            return None

        # 2. Solve the required q1,q2,q3 to achieve wrist center position.
        #    Returns 2 solutions: elbow-up and elbow-down
        q123_sols = self._inverse_position_kinematics(wrist_position.flatten())

        all_solutions = []
        for q1, q2, q3 in q123_sols:
            # 3. Find the necessary rotation
            #    (from initial wrist orientation at solved q1,q2,q3 to goal_orientation)
            #    and determine the required q4,q5,q6 values to achieve that.
            #    Returns 1 or 2 solutions.
            q456_sols = self._inverse_orientation_kinematics(
                goal_orientation, q1, q2, q3, curr_joint_angles[3:]
            )
            for q4, q5, q6 in q456_sols:
                all_solutions.append([q1, q2, q3, q4, q5, q6])

        if len(all_solutions) == 0:
            logger.error("No IK solutions found.")
            return None

        # Filter by joint limits
        all_solutions_arr = np.array(all_solutions)
        valid_solutions = self._filter_by_joint_limits(all_solutions_arr, curr_joint_angles)
        if len(valid_solutions) == 0:
            logger.warning("All IK solutions violate joint limits.")
            return None

        # Select best solution using cost function
        best_idx = self.cost_fn(valid_solutions, curr_joint_angles, self.joint_limits)
        return valid_solutions[best_idx]

    def fk_solve(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Solve forward kinematics (pose of end effector as homogeneous matrix)
        for given joint angles.

        Uses Product of Exponentials with spatial twists
        (See Modern Robotics book for reference)

        Args:
            joint_angles: Shape (6,) - joint angles in radians

        Returns:
            Shape (4,4) homogeneous transformation matrix of end-effector
        """
        q1, q2, q3, q4, q5, q6 = joint_angles

        # Motion of ee_frame due to q6 (wrist roll).
        twist_q6 = np.array([0, 0, 0, 0, 0, -1]) * q6
        T_q6 = expm(self._convert_twist_to_matrix(twist_q6))

        # Motion of ee_frame due to q5 (wrist pitch).
        twist_q5 = np.array([(M.UPPER_ARM + M.FOREARM), 0, 0, 0, -1, 0]) * q5
        T_q5 = expm(self._convert_twist_to_matrix(twist_q5))

        # Motion of ee_frame due to q4 (elbow roll).
        twist_q4 = np.array([0, 0, 0, 0, 0, -1]) * q4
        T_q4 = expm(self._convert_twist_to_matrix(twist_q4))

        # Motion of ee_frame due to q3 (elbow).
        twist_q3 = np.array([-M.UPPER_ARM, 0, 0, 0, 1, 0]) * q3
        T_q3 = expm(self._convert_twist_to_matrix(twist_q3))

        # Motion of ee_frame due to q2 (shoulder).
        twist_q2 = np.array([0, 0, 0, 0, 1, 0]) * q2
        T_q2 = expm(self._convert_twist_to_matrix(twist_q2))

        # Motion of ee_frame due to q1 (turret).
        twist_q1 = np.array([0, 0, 0, 0, 0, 1]) * q1
        T_q1 = expm(self._convert_twist_to_matrix(twist_q1))

        # Total motion of ee_frame
        T = T_q1 @ T_q2 @ T_q3 @ T_q4 @ T_q5 @ T_q6 @ self.initial_T

        # Re-orthogonalize rotation to compensate numerical errors
        T[0:3, 0:3] = self._reorthogonalize_rotation(T[0:3, 0:3])
        return T

    def ik_solve_orientation_only(
        self,
        goal_orientation: np.ndarray,
        curr_joint_angles: np.ndarray,
    ) -> np.ndarray | None:
        """
        Solve inverse kinematics only for a goal orientation of EE frame.
        As there is no goal position, this will keep (q1,q2,q3) same as current and
        solve for (q4,q5,q6).

        This is used for constant EE node.
        """
        q1, q2, q3 = curr_joint_angles[0:3]

        # Find the necessary rotation
        # (from initial wrist orientation at solved q1,q2,q3 to goal_orientation)
        # and determine the required q4,q5,q6 values to achieve that.
        # Returns 1 or 2 solutions.
        curr_wrist_angles = curr_joint_angles[3:]
        q456_sols = self._inverse_orientation_kinematics(
            goal_orientation, q1, q2, q3, curr_wrist_angles
        )

        all_solutions = []
        for q4, q5, q6 in q456_sols:
            all_solutions.append([q1, q2, q3, q4, q5, q6])

        if len(all_solutions) == 0:
            logger.error("No IK solutions found.")
            return None

        # Filter by joint limits
        all_solutions_arr = np.array(all_solutions)
        valid_solutions = self._filter_by_joint_limits(all_solutions_arr, curr_joint_angles)
        if len(valid_solutions) == 0:
            logger.warning("All IK solutions violate joint limits.")
            return None

        # Select best solution using cost function
        best_idx = self.cost_fn(valid_solutions, curr_joint_angles, self.joint_limits)
        return valid_solutions[best_idx]

    # =========================================================================
    # PRIVATE METHODS
    # =========================================================================

    def _inverse_position_kinematics(self, wrist_pos: np.ndarray) -> List[np.ndarray]:
        """
        Solve q1, q2, q3 (turret, shoulder, elbow) for desired wrist position.

        Returns:
            List with [elbow_up, elbow_down] solutions
        """
        px, py, pz = wrist_pos

        # q1 from azimuth
        q1 = math.atan2(py, px)

        # q3 based on required distance ||wrist_pos|| (Law of cosines)
        r = math.sqrt(px**2 + py**2)
        cos_q3 = (r**2 + pz**2 - M.UPPER_ARM**2 - M.FOREARM**2) / (2 * M.UPPER_ARM * M.FOREARM)
        assert -1 <= cos_q3 <= 1

        # Two solutions: elbow up (q3 > 0) and elbow down (q3 < 0)
        q3_up = math.acos(cos_q3)
        q3_down = -q3_up

        solutions = []
        for q3 in [q3_up, q3_down]:
            # q2 to achieve goal elevation
            goal_elevation = math.atan2(pz, r)
            elevation_from_q3 = math.pi / 2 - math.atan2(
                M.FOREARM * math.sin(q3), M.UPPER_ARM + M.FOREARM * cos_q3
            )
            q2 = elevation_from_q3 - goal_elevation

            solutions.append(np.array([q1, q2, q3]))

        return solutions

    def _inverse_orientation_kinematics(
        self,
        goal_R: np.ndarray,
        q1: float,
        q2: float,
        q3: float,
        curr_wrist_angles: np.ndarray,
    ) -> List[np.ndarray]:
        """
        Solve q4, q5, q6 (elbow roll, wrist_pitch, wrist_roll)
        for desired orientation. Returns list of solutions

        Takes the solved q1, q2, q3 from position kinematics and the
        current wrist joint angles (q4,q5,q6) to deal with singularity case.

        Reference: webpage analytical IK from illinois
        """
        # Given (q1,q2,q3), compute the corresponding rotation matrix R_03
        # (orientation of link 3 frame)
        # R_03 = Rz,q1 * Ry,q2 * Ry,q3  => R_03 = Rz,q1 * Ry,q2+q3
        cos_q1 = np.cos(q1)
        sin_q1 = np.sin(q1)
        cos_q23 = np.cos(q2 + q3)
        sin_q23 = np.sin(q2 + q3)
        R_q1 = np.array([[cos_q1, -sin_q1, 0], [sin_q1, cos_q1, 0], [0, 0, 1]])
        R_q23 = np.array([[cos_q23, 0, sin_q23], [0, 1, 0], [-sin_q23, 0, cos_q23]])
        R_03 = R_q1 @ R_q23

        # Obtain the required R_36 from goal_R = R_03 * R_36 * initial_R
        initial_R = self.initial_T[0:3, 0:3]
        R_36 = R_03.T @ goal_R @ initial_R.T

        # Extract (q4, q5, q6) from R_36 = R-z,q4 * R-y,q5 * R-z,q6
        solutions = self._from_R_to_euler(R_36, curr_wrist_angles)

        return solutions

    def _from_R_to_euler(self, R: np.ndarray, curr_wrist_angles: np.ndarray) -> List[np.ndarray]:
        """
        Extract wrist angles (q4, q5, q6) that achieve rotation R.

        Solves for R = Rz(-q4) @ Ry(-q5) @ Rz(-q6), since our wrist joints
        rotate about negative axes (positive q4 rotates EE in -Z direction).

        Singularity: When q5 ≈ 0 or π, the Z axes align and q4 + q6 is
        constrained but not their individual values. We resolve this by
        holding q4 at its current value and solving for q6.

        Reference: Spong, Robot Modeling and Control, p.48 (positive ZYZ extraction)

        Args:
            R: Target rotation matrix
            curr_wrist_angles: Current (q4, q5, q6) for singularity resolution

        Returns:
            List of solutions for [q4, q5, q6] in radians.
            Returns 1 solution at singularity, 2 otherwise.
        """

        ABS_TOL = 1e-3
        solutions = []
        cos_q5 = R[2, 2]

        # Solve for positive ZYZ angles based on Mark Spong reference.
        if np.isclose(cos_q5, 1.0, atol=ABS_TOL):
            # Singularity: q5 = 0, q4 + q6 = atan2(R[1,0], R[0,0])
            # logger.info("Singularity: wrist_pitch = 0")
            q5 = 0.0
            q4 = -curr_wrist_angles[0]
            q6 = normalize_angle(math.atan2(R[1, 0], R[0, 0]) - q4)
            solutions.append(np.array([q4, q5, q6]))

        elif np.isclose(cos_q5, -1.0, atol=ABS_TOL):
            # Singularity: q5 = pi, q4 - q6 = atan2(-R[1,0], -R[0,0])
            # logger.info("Singularity: wrist_pitch = 180")
            q5 = math.pi
            q4 = -curr_wrist_angles[0]
            q6 = normalize_angle(q4 - math.atan2(-R[1, 0], -R[0, 0]))
            solutions.append(np.array([q4, q5, q6]))

        else:
            # General case: two solutions based on sign of sin(q5)
            # Solution 1: sin(q5) < 0
            q4_1 = math.atan2(-R[1, 2], -R[0, 2])
            q5_1 = math.atan2(-math.sqrt(1 - R[2, 2] ** 2), R[2, 2])
            q6_1 = math.atan2(-R[2, 1], R[2, 0])

            # Solution 2: sin(q5) > 0
            q4_2 = math.atan2(R[1, 2], R[0, 2])
            q5_2 = math.atan2(math.sqrt(1 - R[2, 2] ** 2), R[2, 2])
            q6_2 = math.atan2(R[2, 1], -R[2, 0])

            solutions.append(np.array([q4_1, q5_1, q6_1]))
            solutions.append(np.array([q4_2, q5_2, q6_2]))

        # Negate angles to match our model assumption
        for i, sol in enumerate(solutions):
            solutions[i] = -sol

        return solutions

    def _filter_by_joint_limits(
        self,
        q_solutions: np.ndarray,
        q_current: np.ndarray,
        limit_margin: float = np.deg2rad(5),
    ) -> np.ndarray:
        """
        Filter solutions to keep only those within joint limits.

        Args:
            q_solutions: Shape (N, 6) array of candidate solutions
            q_current: Shape (6, ) array that contains current joint angles.
            limit_margin: Safety margin from limits in radians

        Returns:
            Shape (M, 6) array of valid solutions (M <= N)
        """

        # Map joint angles solution from software to physical range
        q_solutions_physical = np.zeros_like(q_solutions)
        for i in range(len(q_solutions)):
            sol_i = q_solutions[i]
            sol_i = map_joint_angles_from_software_to_physical(
                list(sol_i),
                list(q_current),
            )
            q_solutions_physical[i] = np.array(sol_i)

        # Check physical joint limits
        lower = self.joint_limits[0] + limit_margin
        upper = self.joint_limits[1] - limit_margin
        within_limits = (q_solutions_physical >= lower) & (q_solutions_physical <= upper)
        valid_mask = np.all(within_limits, axis=1)

        return q_solutions[valid_mask]

    def _convert_twist_to_matrix(self, twist: np.ndarray) -> np.ndarray:
        """Convert twist (v, w) to 4x4 matrix form."""
        w_hat = self._convert_w_to_matrix(twist[3:6])
        v = twist[0:3, np.newaxis]
        twist_hat = np.hstack((w_hat, v))
        twist_hat = np.vstack((twist_hat, np.zeros((1, 4))))
        return twist_hat

    def _convert_w_to_matrix(self, w: np.ndarray) -> np.ndarray:
        """Converts angular velocity w into its matrix form"""
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def _reorthogonalize_rotation(self, R: np.ndarray) -> np.ndarray:
        """
        Project the matrix `R` to the closest orthogonal matrix (in least square sense)
        through SVD. Useful when composing rotations and
        roundoff errors that drives it away from SO(3).
        """
        U, _, Vt = np.linalg.svd(
            R
        )  # ignore sigma, because orthogonal matrix must have eigenval(R) = 1.
        R = U @ Vt
        return R

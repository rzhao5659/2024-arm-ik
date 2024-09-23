import math
import numpy as np
from scipy.linalg import expm
from typing import Tuple, List

# CONSTANTS: All in meters.
# - INITIAL_T is ee_frame initial configuration w.r.t base_frame
# - UPPER_ARM: upper arm length
# - FOREARM: forearm length
# - TIP_OFFSET_FROM_WRIST_CENTER: End effector tip X-axis offset w.r.t the last frame
#   (wrist_roll / L6 / hand frame)
# - MINIMUM RADIUS: the minimum radius (seen in XY plane) that the goal_position must satisfy
#                   to be valid.
UPPER_ARM = 0.5
FOREARM = 0.5
TIP_OFFSET_FROM_WRIST_CENTER = 0.15
INITIAL_T = np.array(
    [
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [1, 0, 0, UPPER_ARM + FOREARM + TIP_OFFSET_FROM_WRIST_CENTER],
        [0, 0, 0, 1],
    ]
)
MINIMUM_RADIUS = 0.3


def truncate_values_near_zero(matrix, threshold=1e-08):
    return np.where(np.abs(matrix) < threshold, 0.0, matrix)


def ik_solve(
    goal_position: np.ndarray,
    goal_orientation: np.ndarray,
    curr_joint_angles: np.ndarray,
) -> Tuple[bool, List]:
    """
    Solve all 6 joint angles for a 6R arm using analytical IK.
    All modelling assumptions follows the arm model in URDF, without considering joint limits.

    Args:
        goal_position (np.array): a 1D array that denotes the desired end effector position.
        goal_orientation (np.array): a 3x3 rotation matrix that denotes the desired end effector
        orientation.
        curr_joint_angles (np.array): a 1D array that contains the current values of all six
        joints in [-pi, pi]. Required for resolving singularities where joints can have infinite
        solutions.

    Returns:
        boolean, List[float]: boolean to indicate success or fail, then a  list that contains
        the solved joint angles in [-pi;pi].
    """
    # Don't allow a goal_position within a cylinder of radius r centered at base frame origin.
    goal_radius = math.sqrt(goal_position[0] ** 2 + goal_position[1] ** 2)
    if goal_radius <= MINIMUM_RADIUS:
        print("[IK Solver] Fail: Goal radius (on X-Y plane) too close to base frame.\n")
        return False, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Note that the goal position is the end effector / gripper tip desired position,
    # not the wrist position. For non-redundant 6R arm like ours, analytical IK involves two
    # steps:
    # 1) Determining the required wrist position for that goal position:
    #    Since the tip offset is expressed in the wrist roll frame, whose origin
    #    coincides with the wrist center position.  We can express our goal position as:
    #    goal position = wrist position + R * tip_offset.
    #    where R is orientation of wrist_roll frame, expressed in base frame.
    #
    #    Note that the ee_frame is fixed to the wrist_roll frame, it just has an offset
    #    denoted by tip_offset.
    #    Therefore,  R = goal_orientation is given, goal position is given,
    #    the only unknown is wrist_position, which we can solve.
    # 2) Determine the first three joints (turret, shoulder, elbow) to move to our wrist center
    #    to that required wrist center position. This step is called the
    #    inverse position kinematics.
    #
    # 3) Determine the last three joints (elbow roll, wrist_pitch and wrist_roll)
    #    to rotate the wrist so that it aligns with the goal orientation.
    #    This step is called the inverse orientation kinematics.
    #    When that happens, the tip position will also be aligned to the
    #    goal position automatically.

    # Step 1. Find the required wrist center position.
    ee_offset = np.array([TIP_OFFSET_FROM_WRIST_CENTER, 0, 0]).reshape(3, 1)
    wrist_position = goal_position.reshape(3, 1) - goal_orientation @ ee_offset

    # Don't allow a goal_position outside of the workspace of the arm.
    # This can be easily obtained by limiting the wrist center position.
    max_wrist_distance = UPPER_ARM + FOREARM
    if np.linalg.norm(wrist_position) > max_wrist_distance:
        print("[IK Solver] Fail: Goal is out of range.\n")
        return False, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Step 2. With the required wrist center position, solve the required joint angles q1,q2,q3
    # to achieve wrist center position.
    q1, q2, q3 = _inverse_position_kinematics(wrist_position.flatten())

    # Step 3. Find the necessary rotation
    # (from initial wrist orientation at solved q1,q2,q3 to goal_orientation)
    # and determine the required q4,q5,q6 values to achieve that.
    q4, q5, q6 = _inverse_orientation_kinematics(
        goal_orientation, q1, q2, q3, curr_joint_angles[3:6]
    )
    return True, [q1, q2, q3, q4, q5, q6]


def fk_solve(joint_angles: List) -> np.ndarray:
    """
    Solve pose of end effector (homogeneous matrix) given joint angles.
    Use Product of Exponential approach, with spatial twists (v,w) (i.e. expressed in base frame).
    Returns:
        np.array: a 4x4 homogeneous matrix that describes the pose of end effector frame.
    """
    q1, q2, q3, q4, q5, q6 = joint_angles

    # Motion of ee_frame due to q6 (wrist roll).
    twist_q6 = np.array([0, 0, 0, 0, 0, -1]) * q6
    T_q6 = expm(_convert_twist_to_matrix(twist_q6))

    # Motion of ee_frame due to q5 (wrist pitch).
    twist_q5 = np.array([(UPPER_ARM + FOREARM), 0, 0, 0, -1, 0]) * q5
    T_q5 = expm(_convert_twist_to_matrix(twist_q5))

    # Motion of ee_frame due to q4 (elbow roll).
    twist_q4 = np.array([0, 0, 0, 0, 0, -1]) * q4
    T_q4 = expm(_convert_twist_to_matrix(twist_q4))

    # Motion of ee_frame due to q3 (elbow).
    twist_q3 = np.array([-UPPER_ARM, 0, 0, 0, 1, 0]) * q3
    T_q3 = expm(_convert_twist_to_matrix(twist_q3))

    # Motion of ee_frame due to q2 (shoulder).
    twist_q2 = np.array([0, 0, 0, 0, 1, 0]) * q2
    T_q2 = expm(_convert_twist_to_matrix(twist_q2))

    # Motion of ee_frame due to q1 (turret).
    twist_q1 = np.array([0, 0, 0, 0, 0, 1]) * q1
    T_q1 = expm(_convert_twist_to_matrix(twist_q1))

    # Total motion of ee_frame
    T = T_q1 @ T_q2 @ T_q3 @ T_q4 @ T_q5 @ T_q6 @ INITIAL_T
    T = truncate_values_near_zero(T)
    return T


def _inverse_position_kinematics(goal_position: np.ndarray) -> List:
    """
    Solve q1,q2,q3 (turret, shoulder, elbow) based on desired wrist position.
    """
    px = goal_position[0]
    py = goal_position[1]
    pz = goal_position[2]

    # Solve q1 based on goal azimuth.
    q1 = math.atan2(py, px)

    # Solve q3 based on required distance ||goal_position|| (Law of cosines)
    r = math.sqrt(px**2 + py**2)
    cos_q3 = (r**2 + pz**2 - UPPER_ARM**2 - FOREARM**2) / (
        2 * UPPER_ARM * FOREARM
    )
    #  |cos_q1| <= 1 whenever the goal is within arm's reach.
    #  Should always be true, due to goal verification in solve.
    assert -1 <= cos_q3 <= 1
    sin_q3 = math.sqrt(1 - cos_q3**2)
    q3 = math.atan2(sin_q3, cos_q3)

    # Solve q2 to reach goal elevation
    goal_elevation = math.atan2(pz, r)
    elevation_q3 = math.pi / 2 - math.atan2(
        FOREARM * sin_q3, UPPER_ARM + FOREARM * cos_q3
    )  # elevation caused by (q2=0, q3)
    q2 = elevation_q3 - goal_elevation

    return [q1, q2, q3]


def _inverse_orientation_kinematics(
    goal_R: np.ndarray, q1: float, q2: float, q3: float, curr_joint_angles: np.ndarray
) -> List:
    """
    Solve q4, q6, q6 (elbow roll, wrist_pitch, wrist_roll) based on desired end effector
    orientation.
    Takes the solved q1, q2, q3 from position kinematics and the current joint angles
    to deal with singularity case.

    For theory behind this, see webpage analytical IK from illinois
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
    initial_R = INITIAL_T[0:3, 0:3]
    R_36 = np.transpose(R_03) @ goal_R @ np.transpose(initial_R)
    R_36 = truncate_values_near_zero(R_36)

    # Compute the euler angles ZYZ that produces this rotation R_36 = R-z,q4 * R-y,q5 * R-z,q6
    # Passing in negative values because this is solving for positive axes ZYZ, but in our case
    # it needs to be the negative axes.
    # Why ZYZ? See our initial configuration in RVIZ,
    # and see how the last three joints rotates about our base frame axes.
    euler_angles = _from_R_to_euler(R_36, -curr_joint_angles)
    euler_angles = [
        -euler_angles[0],
        -euler_angles[1],
        -euler_angles[2],
    ]
    return euler_angles


def _convert_twist_to_matrix(twist: np.ndarray) -> np.ndarray:
    """Converts twist (v,w) into its matrix form"""
    w_hat = _convert_w_to_matrix(twist[3:6])
    v = twist[0:3, np.newaxis]
    twist_hat = np.hstack((w_hat, v))
    twist_hat = np.vstack((twist_hat, np.zeros((1, 4))))
    return twist_hat


def _convert_w_to_matrix(w: np.ndarray) -> np.ndarray:
    """Converts angular velocity w into its matrix form"""
    wx = w[0]
    wy = w[1]
    wz = w[2]
    return np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])


def _from_R_to_euler(R: np.ndarray, curr_wrist_angles: np.ndarray) -> List:
    """Converts rotation matrix R to ZYZ euler (q4, q5, q6) in range [-pi,pi].
    This set of euler angles encounters singularity when q5 is near 0 or pi,
    which causes ZYZ become ZZ, so q4 and q6 have infinite solutions because
    forearm (L4) and hand (wrist_roll frame / L6) are collinear"
    """
    ABS_TOL = 1e-3
    q4_sol = 0.0
    q5_sol = 0.0
    q6_sol = 0.0

    # Check singularity
    cos_q5 = R[2, 2]
    if _equal(cos_q5, 1, ABS_TOL):
        q5_sol = 0.0
        q4_sol = curr_wrist_angles[0]
        q6_sol = _normalize_angle(math.atan2(R[1, 0], R[0, 0]) - q4_sol)
        print("[IK Solver] Singularity case wrist_pitch = 0 !\n")
    elif _equal(cos_q5, -1, ABS_TOL):
        q5_sol = math.pi
        q4_sol = curr_wrist_angles[0]
        q6_sol = _normalize_angle(q4_sol - math.atan2(-R[1, 0], -R[0, 0]))
        print("[IK Solver] Singularity case wrist_pitch = 180 !\n")
    else:
        # General case (not singularity)
        # Two set of solutions based on sign of q5.
        # Choose the solution that minimizes MAX change in one of the joint values from its
        # current value.
        q4_sol_1 = math.atan2(-R[1, 2], -R[0, 2])
        q5_sol_1 = math.atan2(-math.sqrt(1 - R[2, 2] ** 2), R[2, 2])
        q6_sol_1 = math.atan2(-R[2, 1], R[2, 0])

        q4_sol_1_diff = abs(_get_angle_diff(q4_sol_1, curr_wrist_angles[0]))
        q5_sol_1_diff = abs(_get_angle_diff(q5_sol_1, curr_wrist_angles[1]))
        q6_sol_1_diff = abs(_get_angle_diff(q6_sol_1, curr_wrist_angles[2]))
        max_abs_diff_sol_1 = max(q4_sol_1_diff, q5_sol_1_diff, q6_sol_1_diff)

        q4_sol_2 = math.atan2(R[1, 2], R[0, 2])
        q5_sol_2 = math.atan2(math.sqrt(1 - R[2, 2] ** 2), R[2, 2])
        q6_sol_2 = math.atan2(R[2, 1], -R[2, 0])

        q4_sol_2_diff = abs(_get_angle_diff(q4_sol_2, curr_wrist_angles[0]))
        q5_sol_2_diff = abs(_get_angle_diff(q5_sol_2, curr_wrist_angles[1]))
        q6_sol_2_diff = abs(_get_angle_diff(q6_sol_2, curr_wrist_angles[2]))
        max_abs_diff_sol_2 = max(q4_sol_2_diff, q5_sol_2_diff, q6_sol_2_diff)

        # DEBUG (USEFUL)
        # print(
        #     "Current negative wrist angles (deg)"
        #     f"{convert_angles_from_rad_to_deg(curr_wrist_angles)}"
        # )
        # sol1 = [q4_sol_1, q5_sol_1, q6_sol_1]
        # sol2 = [q4_sol_2, q5_sol_2, q6_sol_2]
        # print(f"Solution 1 (deg){(convert_angles_from_rad_to_deg(sol1))}")
        # print(f"Solution 2 (deg){(convert_angles_from_rad_to_deg(sol2))}")

        if max_abs_diff_sol_1 < max_abs_diff_sol_2:
            q4_sol = q4_sol_1
            q5_sol = q5_sol_1
            q6_sol = q6_sol_1
        else:
            q4_sol = q4_sol_2
            q5_sol = q5_sol_2
            q6_sol = q6_sol_2

    return [q4_sol, q5_sol, q6_sol]


def _get_angle_diff(angle1: float, angle2: float) -> float:
    """
    Get the number of degrees between the two given angles.

    Returns a float in the range [-pi, pi], differences outside this range
    are taken mod 2pi to be within the range.
    """
    diff = (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi
    return diff + 2 * math.pi if diff < -math.pi else diff


def _normalize_angle(angle: float) -> float:
    """
    Normalizes angle to be in range [-pi,pi]
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def _equal(x: float, y: float, abs_tol: float) -> bool:
    """
    Return true if they are equal under the given absolute tolerance.
    """
    return abs(x - y) <= abs_tol

import math
from typing import List

# Constants for the program
PI = math.pi
DEG2RAD = PI / 180
RAD2DEG = 180 / PI


# This file contains functions that deals with angles
# which includes mapping angles from PHYSICAL range to SOFTWARE range, and vice-versa.


def get_angle_diff(angle1: float, angle2: float) -> float:
    """
    Get the number of degrees between the two given angles.

    Returns a float in the range [-pi, pi], differences outside this range
    are taken mod 2pi to be within the range.
    """
    diff = (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi
    return diff + 2 * math.pi if diff < -math.pi else diff


def normalize_angle(angle: float) -> float:
    """
    Normalizes angle to be in range [-pi,pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def convert_angles_from_deg_to_rad(angles: List) -> List:
    result = []
    for q in angles:
        result.append(q * DEG2RAD)
    return result


def convert_angles_from_rad_to_deg(angles: List) -> List:
    result = []
    for q in angles:
        result.append(q * RAD2DEG)
    return result


def map_joint_angles_from_software_to_physical(
    solved_joint_angles: List, curr_joint_angles: List
) -> List:
    """
    Map the solved joint angles from ik_solver, back to their original range,
    and return these as a tuple. Assume all joint angles are in radians, including current joint
    angles.

    Some joint are multi-turn, so we need the current joint angles (in physical ranges)
    to resolve some ambiguity using nearest value.

    This is used before publishing the ik solution as arm command.

    Notations:
    q1, q2, q3, q4, q5, q6 corresponds to
    turret, shoulder, elbow, elbow_roll, wrist_pitch and wrist_roll, respectively.
    """

    sq1, sq2, sq3, sq4, sq5, sq6 = solved_joint_angles
    _, _, _, q4, _, q6 = curr_joint_angles

    # Turret needs to be in the range of [-90, 270].
    sq1 = _map_angle_to_90_270(sq1)

    # Shoulder needs to be in the range of [-90, 90]. Already satisfied.

    # Elbow needs to be an absolute angle from base frame Z axis (vertical)
    # in the range of [-90, 270]
    sq3 = sq3 + sq2
    sq3 = _map_angle_to_90_270(sq3)

    # Elbow roll needs to be in the range of [-360,360].
    # Since it's multiple revolution (so it's a "one-to-many" type mapping),
    # map to the one closest to the current joint angle.
    sq4 = _move_angle_in_shortest_distance(q4, sq4)

    # Wrist pitch is already in the range of [-180,180]. Already satisfied.

    # Wrist roll needs to be in the range of [-inf, inf].
    sq6 = _move_angle_in_shortest_distance(q6, sq6)

    return [sq1, sq2, sq3, sq4, sq5, sq6]


def map_joint_angles_from_physical_to_software(curr_joint_angles: List) -> List:
    """
    Map the measured joint angles, to the range that the fk and ik solvers assume,
    and return these as a tuple. Assume all angles are in radians.

    Notations:
    q1, q2, q3, q4, q5, q6 corresponds to
    turret, shoulder, elbow, elbow_roll, wrist_pitch and wrist_roll, respectively.
    """
    q1, q2, q3, q4, q5, q6 = curr_joint_angles

    # Turret is a relative angle measured in the range of [-90, 270].
    # Positive direction and origin matches with software assumption,
    # so we need to map it to [-180, 180]
    q1 = normalize_angle(q1)

    # Shoulder is a relative angle measured in the range of [-90, 90].
    # Everything matches with software assumption. Do nothing

    # Elbow is an absolute angle from base frame Z axis (vertical), measured in the
    # range of [-90, 270], which means q3 is in reality the sum of q3 relative and q2 relative.
    # We are interested in q3 relative angle.
    # Positive direction and origin matches with software assumption.
    q3 = q3 - q2
    q3 = normalize_angle(q3)

    # Elbow roll is a relative angle measured in the range of [-360,360].
    # Positive direction and origin matches with software assumption
    q4 = normalize_angle(q4)

    # Wrist pitch is a relative angle measured in the range of [-180,180]
    # Everything matches with software assumption. Do nothing

    # Wrist roll is a relative angle measured in the range of [-inf, inf].
    # Positive direction and origin matches with software assumption
    q6 = normalize_angle(q6)

    return [q1, q2, q3, q4, q5, q6]


def _map_angle_to_90_270(q: float):
    """Map an angle [rad] to the range [-pi/2, 2pi/3]"""
    q = normalize_angle(q)
    if -PI < q < -PI / 2:
        q += 2 * PI
    return q


def _move_angle_in_shortest_distance(from_angle: float, to_angle: float) -> float:
    """
    Move from_angle[rad]([-inf,inf]) to the position that to_angle[rad]([-inf,inf])
    represents in the unit circle, in the shortest distance possible, and return this angle.

    For example,
    if from_angle is 750 degrees (which represents 30 degrees), and to_angle is 10 degrees,
    this function will return the angle 730 degrees, which is the same position as to_angle,
    but at the particular revolution of from_angle.

    This function is used for mapping angles for joints that are multi-turn.
    """
    # 1. Get the difference in [-pi,pi] between from_angle and to_angle in the unit circle.
    diff_angle = normalize_angle(
        normalize_angle(to_angle) - normalize_angle(from_angle)
    )
    # 2. Add this difference to the from_angle.
    final_angle = from_angle + diff_angle
    return final_angle

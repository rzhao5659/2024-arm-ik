import numpy as np

# ==== Dimensions and Pose of Arm [meters] ====
# - INITIAL_T is ee_frame initial pose w.r.t base_frame
# - UPPER_ARM: upper arm length
# - FOREARM: forearm length
# - TIP_OFFSET_FROM_WRIST_CENTER: End effector tip X-axis offset w.r.t the last frame
#   (wrist_roll / hand frame)
UPPER_ARM = 0.5
FOREARM = 0.5 + 0.025
TIP_OFFSET_FROM_WRIST_CENTER = 0.15

# ===== Physical joint limits constants [deg] =====
JOINTS_LOWER_LIMITS = (
    -90,  # turret
    -90,  # shoulder
    -90,  # elbow
    -180,  # elbow_roll
    -90,  # wrist_pitch
    -np.inf,  # wrist_roll
)


JOINTS_UPPER_LIMITS = (
    270,  # turret
    90,  # shoulder
    270,  # elbow
    180,  # elbow_roll
    90,  # wrist_pitch
    np.inf,  # wrist_roll
)

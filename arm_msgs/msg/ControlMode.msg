# This message type has just a singular field: `mode`, which is an unsigned integer.

uint8 mode


# The interpretation of this field is "one of" the constants defined below.
# Any other integer has no meaning or valid interpretation.

# standard velocity and position modes are in the 1-9 range
uint8 MODE_JOY_BASE_JOY_END = 1 # velocity (base and end controlled by joysticks)
uint8 MODE_MODEL_ARM_JOY_END = 2 # position (model arm for base, joystick for end)
uint8 MODE_ADVANCED = 3

# IK modes are in the 10s (11-19)
uint8 MODE_IK_CHASSIS_FRAME = 11 # control goal XYZ PRY in the chassis frame of the end effector
uint8 MODE_IK_GRIPPER_FRAME = 12 # control goal XYZ PRY in the end effector frame of the end effector
uint8 MODE_IK_CONSTANT_EE = 13 # model arm is used for goal XYZ of end effector, IK for constant orientation of end effector

# preset modes are in the 20s (21-29)
uint8 MODE_PRESETS = 21

# stopping modes are in the 100s (101+)
uint8 MODE_STOP = 101
uint8 MODE_DISABLE = 102
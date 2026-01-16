
## Overview

This packages contains
- `solvers.py` contains IK and FK solvers for 6R arm.
- `local_ik_node.py` and `constant_ee_ik_node.py` are nodes that uses solvers to control arm.


The `local_ik_node.py` allows moving arm's end effector pose (XYZ translation or rotation) with respect to either **base frame** or **end effector frame**.

The `constant_ee_ik_node.py` allows translating arm's end effector using our physical model arm, while internally IK maintains the current orientation of end effector.


### Local IK Node Detail

This node tracks a reference pose `reference_T` and increment variables `(dx, dy, dz, droll, dpitch, dyaw)`.

  - When the arm enters IK mode, it calls `fk_solve` once to compute the current pose as `reference_T`.

  - `IKInput` messages update increment variables which are converted to an incremental transformation dT.

  - Target pose is then derived as `target_T = reference_T @ dT`


This node has two behaviors based on control mode:
- When arm is in IK mode, the IK solve for goal pose. This is done internally by setting
  - `reference_T` = pose when entering IK mode
  - Increments (`dx,dy,dz,droll,dpitch,dyaw`) accumulate from IKInput messages

- When arm is not in IK mode, the IK solve for current pose.

  This allows an external node to validate safety (solved joint angles $\approx$ current joint angles ) and enter IK mode safely.
  This is done internally by constantly setting
  - `reference_T` = current pose
  - Increments = 0


## Usage

Run local IK node
```
ros2 run arm_ik local_ik_node
```

Run constant EE IK node
```
ros2 run arm_ik local_ik_node
```

Runs both
```
ros2 launch arm_ik arm_ik_launch.xml
```


## Modelling Assumptions For Solvers

Launch this to visualize the URDF that displays the software modelling assumptions in RVIZ.

The solvers assume a range of [-180, 180] in radians for all measured joint angles.

```bash
ros2 launch arm_ik display.launch.py gui:=true
```

Summary of assumptions:

- Initial pose exactly as rviz shown (all joints are 0 at that pose)
- Assume positive thetas for these directions:
  - turret: base frame z axis (CCW)
  - shoulder moves downward
  - elbow moves downward.
  - elbow roll CCW
  - wrist pitch upward.
  - wrist roll CCW.
- All joint angles are relative to their parent frame/link.
- All joints angles are in the range of [-180,180]

## Test

### Test Local IK Node without the arm

```bash
ros2 launch arm_ik display.launch.py gui:=False
ros2 run arm_ik local_ik_node
ros2 run arm_ik ik_node_test --ros-args -p ik_topic:=/arm/command/ik
rqt
```

- Use rqt to set up a Message Publisher panel for:
  - `IKInput` at `/arm/ik_input/filtered`.
  - `ControlMode` at `/arm/mode/current`.
- Verify visually that motion matches expectation.

Note: This doesn't test the multiturn aspect of the joints like elbow roll and wrist roll (so they all remain in [-180,180])

### Test Constant EE without the arm
```bash
ros2 launch arm_ik display.launch.py gui:=False
ros2 run arm_ik constant_ee_ik_node
ros2 run arm_ik ik_node_test --ros-args -p ik_topic:=/arm/command/ik_constant_ee
rqt
```

- Use rqt to set up a Message Publisher panel for:
  - `ModelArmState` at `/arm/model/raw`.
  - `ControlMode` at `/arm/mode/current`.
- Verify visually that motion matches expectation.

### Test IK with the arm

These two nodes can help testing local_ik_node (or constant_ee_ik_node) with the real arm.

Run `ros2 launch arm_ik display.launch.py gui:=False`, then either
- `ros2 run arm_ik arm_status_rviz` will display the arm in RVIZ following ArmStatus joint angles. It's how the arm will look like at the measured joint angles.
- `ros2 run arm_ik ik_command_rviz --ros-args -p ik_topic:=/arm/command/ik` will display the arm in RVIZ following IK Command joint angles. It's how the arm will look at the commanded joint angles.

## Appendix

### Future Work

- Self-collision detection:

  The arm currently need to use conservative joint limits (stricter than actual physical ones) because it has no self-collision detection.

  We have the meshes of each arm link, and we have a FK solver that can return pose of each arm link. One future improvement is to write a self-collision detection routine that may allow more movement.

### Local IK Design

Q: Why do we have a `reference_T` and increment variables?

An alternative way to implement local IK node is to discard `reference_T` and increment variables `(dx, dy, dz, droll, dpitch, dyaw)`, and use this approach:

Everytime we receive `IKInput` command message, from which we extract the increments, we call `fk_solve` on current joint angles to get current EE pose, then computes goal pose, then call `ik_solve` to get the goal joint angles.

This simpler approach has these disadvantages:
1. Computationally more expensive (must call FK every iteration, vs FK once to get `reference_T` )
2. This approach goal pose `Tgoal` may not be what we expect:

    Even a simple IK command (e.g., +0.1 m to the right) doesn’t guarantee the EE moves strictly along that direction (although we do optimize for minimum joint motion), since joint paths aren’t explicitly planned.

    If the EE orientation changes mid-motion and we recompute IK using the current pose, the resulting target could differ. Using a **fixed initial reference_T** ensures all IK outputs remain consistent increments relative to the same starting pose, instead of using a dynamically changing current EE pose.
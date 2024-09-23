## Description

Local IK allows moving arm's end effector locally either with respect to **base frame** or **end effector frame**.

- When the arm enters IK mode, the node records the current pose (`ArmStatus` at `/arm/status/current`) as the **reference pose**.
- The node listens and accumulates 3D Mouse message (`MouseState` at `/arm/mouse/filtered`) as increments from this **reference pose**.
- Everytime 3D Mouse message is received, IK is solved on **goal pose** = reference pose + accumulated increments.

  - If the solved joint angles are verified to be safe, then these are published as IK command (`ArmCommand` at `/arm/command/ik`).
  - Otherwise, IK command is not published and the increments from this MouseState message is discarded.

- The solved joint angles are verified to be safe if:

  - IK solver was successful in solving them: Goal pose wasn't out of reach for the arm, or too close to base frame origin.
  - They satisfy joint limits in `local_ik_node.py`

Change of frame for increments:

- The increments can be interpreted as relative to **base frame** or **end effector frame**. This can be changed via the service (`LocalIKFrameRequest` at `/arm/command/ik/frame`). For example, if your MouseState is set to `x = 1.0`, then the end effector will move in X axis of that frame.

- Whenever a change of frame is requested, this will recompute the **reference pose** and reset **accumulated increments** to 0, effectively restarting Local IK at that pose.

## Usage

Runs local IK node
```
ros2 run arm_ik local_ik_node
```

**Configurable parameters**:
The joint limits and the sensitivity to `MouseState` are configurable in `local_ik_node.py`.



## Notes

Our 6-DoF arm can suffer from singularity when elbow roll and wrist roll axes are collinear, such that the wrist loses one degree of freedom.  When this happens, and the user indicate it to move toward the direction it can't move, then it will jump in configuration to allow that movement. 


## Modelling Assumptions For Solvers

Launch this to visualize the URDF (which contains almost all modelling assumptions) in RVIZ.
The solvers additionally assume a range of [-180, 180] for all measured joint angles.

```bash
ros2 launch arm_ik display.launch.py
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
ros2 run arm_ik local_ik_test
ros2 run arm_ik local_ik_node
rqt
```

The local ik test doesn't test the multiturn aspect of the joints like elbow roll and wrist roll (so they all remain in [-180,180])

- Use rqt to set up a Message Publisher panel for:
  - `MouseState` at `/arm/mouse/filtered`.
  - `ControlMode` at `/arm/mode/current`.
- Use rqt to set up a Service Caller panel for:
  - `LocalIKFrameRequest` at `/arm/command/ik/frame`.
- Verify visually that motion matches expectation.
- Safety cases are already tested: https://www.notion.so/nurover/ARM_IK-Testing-safety-failure-cases-dcb64601bf8f492faf5a7219a9f82f9c?pvs=4

### Test Local IK node with the arm

These two nodes can help testing local_ik_node with the real arm.

Run `ros2 launch arm_ik display.launch.py gui:=False`, then either

- `ros2 run arm_ik arm_status_rviz` will display the arm in RVIZ following ArmStatus joint angles. It's how the arm will look like at the measured joint angles.
- `ros2 run arm_ik ik_command_rviz` will display the arm in RVIZ following IK Command joint angles. It's how the arm will look at the commanded joint angles.

Note: at the moment you can only run one of them and have one instance of RVIZ.
These two can be useful for testing with the ik with the real arm.

#### Test IK solver [Optional]

The Local IK node requires both IK solver and FK solver to work properly, so there are optional tests for those in isolation.

The IK Solver does't consider any joint limit.

```bash
ros2 launch arm_ik display.launch.py gui:=False
ros2 run arm_ik ik_solve_test
rqt
```

- Use rqt to set up a Message Publisher panel to publish `MouseState` at `/test/ik_cmd`. This is interpreted as goal pose expressed in base frame, and not as increments.

- Put any values in the xyzrpy fields, you should see the ee_frame in rviz to be exactly at that pose. \
  If it doesnt move, then you should see in the terminal where you executed ik_solve_test that Success = False, due to goal_position being invalid.

#### Test FK solver [Optional]

```bash
ros2 launch arm_ik display.launch.py
ros2 run arm_ik fk_solve_test
rqt
```

- Use the `joint_state_gui` to input joint values.
- See that the transformation matrix printed matches exactly as `ros2 run tf2_ros tf2_echo base_frame ee_frame`

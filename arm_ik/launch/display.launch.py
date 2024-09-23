import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

package_name = "arm_ik"
package_share_path = get_package_share_directory(package_name)
urdf_path = os.path.join(package_share_path, "urdf", "Articulated_Arm_Rev2.urdf")
rviz_config_path = os.path.join(package_share_path, "rviz", "urdf.rviz")


def generate_launch_description():
    params = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("model")]), value_type=str
        )
    }

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="This is a flag for joint_state_publisher_gui",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=urdf_path,
                description="Path to the urdf model file",
            ),
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )

import os

import launch
import launch_ros
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

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
    arm_status_node = launch_ros.actions.Node(
        package="arm_ik",
        executable="arm_status_rviz",
        name="arm_status_rviz",
        output="screen",
        condition=LaunchConfigurationEquals("input", "arm_status"),
    )
    ik_command_node = launch_ros.actions.Node(
        package="arm_ik",
        executable="ik_command_rviz",
        name="ik_command_rviz",
        output="screen",
        condition=LaunchConfigurationEquals("input", "ik"),
        parameters=[{"ik_topic": "/arm/command/ik"}],
    )
    constant_ee_ik_command_node = launch_ros.actions.Node(
        package="arm_ik",
        executable="ik_command_rviz",
        name="ik_command_rviz",
        output="screen",
        condition=LaunchConfigurationEquals("input", "constant_ee"),
        parameters=[{"ik_topic": "/arm/command/ik_constant_ee"}],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="gui",
                default_value="False",
                description="This is a flag for joint_state_publisher_gui",
            ),
            launch.actions.DeclareLaunchArgument(
                name="input",
                default_value="",
                description="Input source to show in rviz",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=urdf_path,
                description="Path to the urdf model file",
            ),
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
            arm_status_node,
            ik_command_node,
            constant_ee_ik_command_node,
        ]
    )

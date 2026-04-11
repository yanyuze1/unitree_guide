import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("unitree_guide")
    mujoco_launch = os.path.join(pkg_share, "launch", "mujoco.launch.py")
    topic_guide = os.path.join(pkg_share, "config", "plotjuggler_trotting_yaw_debug_topics.md")

    launch_stack = LaunchConfiguration("launch_stack")
    launch_plotjuggler = LaunchConfiguration("launch_plotjuggler")
    pkg_description = LaunchConfiguration("pkg_description")
    buffer_size = LaunchConfiguration("buffer_size")

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_stack",
            default_value="false",
            description="Launch the controller stack together with PlotJuggler.",
        ),
        DeclareLaunchArgument(
            "launch_plotjuggler",
            default_value="true",
            description="Launch PlotJuggler in a separate process.",
        ),
        DeclareLaunchArgument(
            "pkg_description",
            default_value="go2_description",
            description="Package that provides robot description and controller parameters.",
        ),
        DeclareLaunchArgument(
            "buffer_size",
            default_value="40",
            description="PlotJuggler streaming buffer size in seconds.",
        ),
        LogInfo(msg=["PlotJuggler topic guide: ", topic_guide]),
        LogInfo(
            msg=(
                "Install PlotJuggler first if needed: "
                "sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mujoco_launch),
            condition=IfCondition(launch_stack),
            launch_arguments={"pkg_description": pkg_description}.items(),
        ),
        ExecuteProcess(
            condition=IfCondition(launch_plotjuggler),
            cmd=[
                "ros2",
                "run",
                "plotjuggler",
                "plotjuggler",
                "-n",
                "--buffer_size",
                buffer_size,
                "--window_title",
                "unitree_trotting_yaw_debug",
            ],
            output="screen",
        ),
    ])

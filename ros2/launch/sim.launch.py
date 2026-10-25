"""Bring up the whole stack against the virtual STM32 with one command:

    ros2 launch mdp_ros2 sim.launch.py

then, in another terminal:   ros2 service call /plan std_srvs/srv/Trigger
                              ros2 service call /run_mission std_srvs/srv/Trigger

NOT YET RUN: written, not launched.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    virtual_stm = ExecuteProcess(
        cmd=["ros2", "run", "mdp_ros2", "virtual_stm32_pty", "--link", "/tmp/vstm32", "--speed", "4"],
        output="screen",
    )
    nodes = [
        Node(package="mdp_ros2", executable="stm32_bridge_node", name="stm32_bridge",
             parameters=[{"port": "/tmp/vstm32"}], output="screen"),
        Node(package="mdp_ros2", executable="mdp_planner_node", name="mdp_planner", output="screen"),
        Node(package="mdp_ros2", executable="mission_node", name="mission", output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ]
    # Give the pseudo-terminal a moment to appear before the driver opens it.
    return LaunchDescription([virtual_stm, TimerAction(period=1.0, actions=nodes)])

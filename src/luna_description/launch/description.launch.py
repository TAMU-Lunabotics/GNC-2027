"""Publish the robot description and its TF (base_link -> chassis, wheels, sensor frames).

  ros2 launch luna_description description.launch.py
  ros2 launch luna_description description.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('luna_description')
    xacro_file = os.path.join(share, 'urdf', 'luna.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' geometry:=', LaunchConfiguration('geometry')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('geometry',
                              default_value=os.path.join(share, 'config', 'robot_geometry.yaml')),
        Node( # this node handles setting up the transforms for our robot based off of our urdf
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
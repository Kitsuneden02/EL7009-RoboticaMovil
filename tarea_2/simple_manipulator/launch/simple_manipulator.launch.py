from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command

from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(Command(['xacro ', PathJoinSubstitution([FindPackageShare('simple_manipulator'), 
                                                'urdf', 'simple_manipulator.xacro'])]), value_type=str)

    rviz_config_file = PathJoinSubstitution([FindPackageShare('simple_manipulator'), 
                                                'launch', 'simple_manipulator.rviz'])
                            
    launch_description = LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            namespace='',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            namespace='',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])

    return launch_description

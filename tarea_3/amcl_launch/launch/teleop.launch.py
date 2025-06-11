import os
from launch.frontend.parser import parse_if_substitutions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name='amcl_launch'

    # Launch configurations

    map_yaml = os.path.join(get_package_share_directory(package_name),'map','mapa.yaml')

    # Launch the amcl node
    amcl_params = os.path.join(get_package_share_directory(package_name),'config','amcl.yaml')
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params],
        
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )
     
    teleop_twist_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{'axis_linear.x': 1, 'axis_angular.yaw': 0, 'scale_linear': 1.1, 'scale_angular': 1.0}],
        remappings=[('cmd_vel', 'cmd_vel')],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments

        # Launch the nodes
        joy_node,
        teleop_twist_node,
    ])

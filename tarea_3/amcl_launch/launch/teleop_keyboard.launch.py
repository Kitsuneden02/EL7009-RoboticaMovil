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

     
    teleop_twist_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        # parameters=[{ 'scale_linear': 1.1, 'scale_angular': 1.0}],
        remappings=[('cmd_vel', 'cmd_vel')],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments

        # Launch the nodes
        # joy_node,
        teleop_twist_node,
    ])

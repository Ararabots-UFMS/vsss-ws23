from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration


def generate_launch_description():
    
    interface = LaunchConfiguration('interface')
    side = LaunchConfiguration('side')
    color = LaunchConfiguration('color')
    n_robots = LaunchConfiguration('n_robots')

    interface_launch_arg = DeclareLaunchArgument(
        'interface',
        default_value='True'
    )

    side_launch_arg = DeclareLaunchArgument(
        'side',
        default_value='left'
    )

    color_launch_arg = DeclareLaunchArgument(
        'color',
        default_value='yellow'
    )
    n_robots_launch_arg = DeclareLaunchArgument(
        'n_robots',
        default_value='3'
    )

    return LaunchDescription([
        interface_launch_arg,
        side_launch_arg,
        color_launch_arg,
        n_robots_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_files'),
                    'quick_simulation_launch.py'
                ])
            ]),
            launch_arguments={
                'interface': interface,
                'side': side,
                'color': color,
                'n_robots': n_robots
            }.items()
        )
    ])
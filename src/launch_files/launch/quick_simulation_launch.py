from multiprocessing.sharedctypes import Value
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from sys import argv
import platform

param_dict_side = {
    "direita":'0',
    "esquerda":'1',
    "right":'0',
    "left":'1'
}

param_dict_color = {
    "azul": '0',
    "amarelo" : '1',
    "blue": '0',
    "yellow" : '1'
}


def evaluate_params(context, *args, **kwargs):

    user_namespace = platform.node().replace('-','_')
    interface = LaunchConfiguration('interface').perform(context)
    side = LaunchConfiguration('side').perform(context)
    color = LaunchConfiguration('color').perform(context)
    n_robots = LaunchConfiguration('n_robots').perform(context)

    try:
        interface = interface == 'True' 
    except ValueError as exception:
        print(exception)
        interface = True

    try:
        side = param_dict_side[side]
    except ValueError as exception:
        print(exception)
        raise exception

    try:
        color = param_dict_color[color]
    except ValueError as exception:
        print(exception)
        raise exception

    launch_nodes = [
        Node(
            package='sim_cam',
            # namespace='user',
            executable='publisher',
            name='sim_cam'
        ),
        Node(
            package='sim_message_server',
            namespace=user_namespace,
            executable='MessageServerNode',
            name='message_server',
            arguments=[user_namespace, side, color]
        )
    ]
    if interface:
        interface_node = Node(package='interface',
                namespace=user_namespace,
                executable='InterfaceNode',
                name='interface',
                arguments=[user_namespace]
        )
        launch_nodes.append(interface_node)
        launch_nodes.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=interface_node,
                    on_exit=[
                        LogInfo(msg=(user_namespace,
                                ' closed the main window')),
                        EmitEvent(event=Shutdown(
                            reason='Window closed'))
                    ]
                )
            )
        )
    else:
        launch_nodes.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=launch_nodes[0],
                    on_exit=[
                        LogInfo(msg=(user_namespace,
                                ' closed the main window')),
                        EmitEvent(event=Shutdown(
                            reason='Window closed'))
                    ]
                )
            )
        )
        for robot_index in range(int(n_robots)):
            robot_index = str(robot_index)
            launch_nodes.append(
                Node(
                    package='robot',
                    namespace=user_namespace,
                    executable='RobotNode',
                    name='ROBOT_'+robot_index,
                    arguments=[robot_index, robot_index, 'simulation_fixed_Kp', side, color, "0", "user", robot_index, '0']
                )
            )

    return launch_nodes

def generate_launch_description():
    interface_launch_arg = DeclareLaunchArgument(
        'interface',
        default_value='False'
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

    return LaunchDescription([interface_launch_arg, side_launch_arg, color_launch_arg, n_robots_launch_arg, OpaqueFunction(function=evaluate_params)])

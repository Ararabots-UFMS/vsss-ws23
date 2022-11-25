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

message_server_dict = {
    "esp_now" : "now_message_server",
    "ble" : "ble_message_server",
    "bt_classic" : "message_server"
}

def evaluate_params(context, *args, **kwargs):

    user_namespace = platform.node().replace('-','_')
    message_server_arg = LaunchConfiguration('message_server')

    try:
        message_server = message_server_dict[message_server_arg]
    except ValueError as exception:
        print(exception)
        message_server = "now_message_server"

    launch_nodes = [
        Node(
            package='vision',
            # namespace='user',
            executable='publisher',
            name='vision'
        ),
        Node(
            package=message_server
            namespace=user_namespace,
            executable='MessageServerNode',
            name='message_server',
            arguments=[user_namespace]
        )
    ]

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

    return launch_nodes

def generate_launch_description():

    message_server_arg = DeclareLaunchArgument(
        'message_server',
        default_value='esp_now',
        description="Options: [esp_now, ble, bt_classic]"
    )
    
    return LaunchDescription([message_server_arg, OpaqueFunction(function=evaluate_params)])

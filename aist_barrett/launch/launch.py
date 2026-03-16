from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.descriptions    import ComposableNode
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution(
                           [FindPackageShare('aist_fastening_tools'),
                            'config', 'default.yaml']),
        'description': 'absolute path to configuration file'
    },
    {
        'name':        'name',
        'default':     'barrett_hand',
        'description': 'device name'
    },
    {
        'name':        'container',
        'default':     'bhand_container',
        'description': 'name of the component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def launch_setup(context):
    composable_nodes = [
        ComposableNode(
            name=LaunchConfiguration('name'),
            package='aist_barrett',
            plugin='aist_barrett::BarrettHandController',
            parameters=[LaunchConfiguration('param_file')],
            extra_arguments=[{'use_intra_process_comms': True}])
    ]

    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=composable_nodes)
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])

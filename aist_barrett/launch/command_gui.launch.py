from launch                            import LaunchDescription
from launch.actions                    import (OpaqueFunction,
                                               IncludeLaunchDescription)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'device_name',
        'default':     'bhand',
        'description': 'device name'
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
    param_file = ParameterFile(
                     PathJoinSubstitution([FindPackageShare('aist_barrett'),
                                           'config', 'default.yaml']),
                     allow_substs=True)
    return [
        Node(namespace='gui',
             package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             parameters=[param_file],
             remappings=[
                 ('robot_description', '/robot_description'),
             ]),
        Node(package='aist_barrett',
             executable='joint_state_to_array.py',
             parameters=[param_file],
             remappings=[
                 ('joint_states', 'gui/joint_states'),
                 ('~/out',        [LaunchConfiguration('device_name'),
                                   '_controller/commands']),
             ],
             output=LaunchConfiguration('output')),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])

from launch                            import LaunchDescription
from launch.actions                    import (OpaqueFunction,
                                               IncludeLaunchDescription)
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'device_name',
        'default':     'bhand',
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
    param_file = ParameterFile(
                     PathJoinSubstitution([FindPackageShare('aist_barrett'),
                                           'config', 'default.yaml']),
                     allow_substs=True)
    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [FindPackageShare('aist_barrett'),
                                          'urdf', 'barrett_hand.urdf'])]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 param_file,
                 {'robot_description': robot_description}
             ],
             output=LaunchConfiguration('output')),
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
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_barrett'), 'launch',
                                  'launch.py'])),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([FindPackageShare('aist_barrett'),
                                       'launch', 'aist_barrett.rviz'])
             ]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])

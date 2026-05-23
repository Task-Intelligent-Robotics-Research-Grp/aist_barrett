from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               Command, FindExecutable)
from launch.conditions                 import IfCondition
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue

launch_arguments = [
    {
        'name':        'joint_gui',
        'default':     'true',
        'description': 'Launch joint_state_publisher_gui if true',
        'choices':     ['true', 'false', 'True', 'False']
    }
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    robot_description = ParameterValue(
                            Command([
                                FindExecutable(name='xacro'), ' ',
                                PathJoinSubstitution([
                                    ThisLaunchFileDir(), '..', 'urdf',
                                    'barrett_hand.urdf'
                                ])
                            ]),
                            value_type=str)
    return [Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description}]),
            Node(package='joint_state_publisher_gui',
                 executable='joint_state_publisher_gui',
                 condition=IfCondition(LaunchConfiguration('joint_gui'))),
            Node(name='rviz', package='rviz2', executable='rviz2',
                 output='screen',
                 arguments=['-d',
                            PathJoinSubstitution([ThisLaunchFileDir(),
                                                  'aist_barrett.rviz'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])

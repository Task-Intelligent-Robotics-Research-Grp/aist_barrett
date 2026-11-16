from launch                            import LaunchDescription
from launch.actions                    import (OpaqueFunction,
                                               IncludeLaunchDescription)
from launch.substitutions              import (Command, FindExecutable,
                                               PathJoinSubstitution,
                                               ThisLaunchFileDir,
                                               IfElseSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue

launch_arguments = [
    {
        'name':        'debug',
        'default':     'false',
        'description': 'Supress opening another terminal window if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
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
                                    FindPackageShare('aist_barrett'), 'urdf',
                                    'barrett_hand.urdf'
                                ]),
                                ' mimic_outer_joints:=false',
                            ]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'robot_description': robot_description}
             ],
             output='screen'),
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'launch.py'])),
        Node(name=['test_client'],
             package='aist_barrett',
             executable=['test_client.py'],
             prefix=[
                 IfElseSubstitution(
                     LaunchConfiguration('debug'),
                     if_value='',
                     else_value='gnome-terminal --tab --wait --active --')
             ],
             output='screen'),
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

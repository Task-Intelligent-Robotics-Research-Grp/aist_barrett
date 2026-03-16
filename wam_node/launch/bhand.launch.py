from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
]

def launch_setup(context):
    # Create robot description from URDF.
    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [FindPackageShare('wam_description'),
                                          'urdf', 'barrett_hand.urdf'])]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'robot_description': robot_description}
             ],
             output='screen'),
        Node(name='wam_node',
             package='wam_node',
             executable='wam_node',
             output='screen'),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([
                     FindPackageShare('wam_node'), 'launch', 'wam_node.rviz',
                 ])
             ]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])

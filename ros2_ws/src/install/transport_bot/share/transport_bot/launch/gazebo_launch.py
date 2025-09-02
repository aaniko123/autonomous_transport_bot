from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('transport_bot'),
        'urdf',
        'transport_bot.urdf.xacro'
    ])

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher (reads robot_description from processed xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ', urdf_file
                ])
            }],
            output='screen'
        ),

        # Spawn the robot using topic (robot_description)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'transport_bot',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])

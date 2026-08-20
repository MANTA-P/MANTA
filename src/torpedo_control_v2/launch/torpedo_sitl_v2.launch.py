from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_config = PathJoinSubstitution(
        [FindPackageShare('torpedo_control_v2'), 'config', 'bridge.yaml']
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='torpedo_ros_gz_bridge_v2',
        output='screen',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': use_sim_time},
        ],
    )

    controller = Node(
        package='torpedo_control_v2',
        executable='torpedo_control_node_v2',
        name='torpedo_control_node_v2',
        output='screen',
        emulate_tty=True,
        output_format="{line}",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock',
            ),
            bridge,
            controller,
        ]
    )

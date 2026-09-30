from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    device = LaunchConfiguration('device')
    baud_rate = LaunchConfiguration('baud_rate')
    armed = LaunchConfiguration('armed')
    mode = LaunchConfiguration('mode')
    target_thrust = LaunchConfiguration('target_thrust')

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

    esp_bridge = Node(
        package='esp32_bridge',
        executable='esp32_ros_uart_tx_node',
        name='esp32_torpedo_hil_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device': device,
            'baud_rate': baud_rate,
            'control.armed': armed,
            'control.mode': mode,
            'control.target_thrust': target_thrust,
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock',
            ),
            DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
            DeclareLaunchArgument('baud_rate', default_value='921600'),
            DeclareLaunchArgument('armed', default_value='false'),
            DeclareLaunchArgument('mode', default_value='0'),
            DeclareLaunchArgument('target_thrust', default_value='0'),
            bridge,
            esp_bridge,
        ]
    )

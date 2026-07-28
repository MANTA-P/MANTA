from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("bluerov_integration")
    bridge_config = os.path.join(package_share, "config", "bridge.yaml")
    integration_config = os.path.join(
        package_share, "config", "integration.yaml"
    )
    rviz_config = os.path.join(
        package_share, "config", "bluerov_analysis.rviz"
    )
    start_bridge = LaunchConfiguration("start_bridge")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the package BlueROV ros_gz_bridge",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Open the full analysis RViz view",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="bluerov_parameter_bridge",
            parameters=[{"config_file": bridge_config}],
            output="screen",
            condition=IfCondition(start_bridge),
        ),
        Node(
            package="bluerov_integration",
            executable="bluerov_integration_node",
            name="bluerov_integration_node",
            parameters=[
                integration_config,
                {
                    "planning.enabled": True,
                    "control.enabled": False,
                    "control.logging.enabled": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="bluerov_integration_rviz",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(start_rviz),
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("ros2_imu_serial"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_imu_serial",
                executable="imu_serial_node",
                name="imu_serial_node",
                output="screen",
                parameters=[params_file],
                remappings=[("imu/data_raw", "imu/data_raw")],
            )
        ]
    )

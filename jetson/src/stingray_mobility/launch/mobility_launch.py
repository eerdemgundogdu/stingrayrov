import os
from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore
from launch_ros.actions import Node  # type: ignore
from ament_index_python.packages import get_package_share_directory  # type: ignore

def generate_launch_description():
    pkg_share = get_package_share_directory('stingray_mobility')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Teensy USB Serial port yolu'
    )

    serial_bridge = Node(
        package='stingray_mobility',
        executable='serial_bridge',
        name='serial_bridge_node',
        output='screen',
        parameters=[
            params_file,
            {'serial_port': LaunchConfiguration('serial_port')}
        ]
    )

    sensor_node = Node(
        package='stingray_mobility',
        executable='sensor_node',
        name='sensor_node',
        output='screen',
        parameters=[params_file]
    )

    controller = Node(
        package='stingray_mobility',
        executable='controller',
        name='controller_node',
        output='screen',
        parameters=[params_file]
    )

    mission = Node(
        package='stingray_mobility',
        executable='mission',
        name='mission_node',
        output='screen',
        parameters=[params_file]
    )

    safety = Node(
        package='stingray_mobility',
        executable='safety',
        name='safety_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        serial_port_arg,
        serial_bridge,
        sensor_node,
        controller,
        mission,
        safety,
    ])


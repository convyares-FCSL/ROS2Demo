from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    server = Node(
        package='tank_pressure_demo_cpp',
        executable='fill_action_server',
        name='fill_action_server',
        output='screen',
        parameters=[{
            'start_bar': 96.0,
            'tick_ms': 100,
            # choose one:
            # 'ramp_bar_per_s': 1.0,
            'aprr_mpa_per_min': 1.5,  # overrides ramp_bar_per_s if > 0
        }],
    )

    logger = Node(
        package='tank_pressure_tools_py',
        executable='pressure_logger',
        name='pressure_logger',
        output='screen',
        parameters=[{
            'csv_path': '/tmp/tank_pressure.csv'
        }],
    )

    client = Node(
        package='tank_pressure_demo_cpp',
        executable='fill_action_client',
        name='fill_action_client',
        output='screen',
        arguments=['705.0'],  # target bar
    )

    return LaunchDescription([server, logger, client])

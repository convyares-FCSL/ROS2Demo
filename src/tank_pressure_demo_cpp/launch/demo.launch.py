from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tank_pressure_demo_cpp",
            executable="pressure_publisher",
            name="pressure_publisher",
            output="screen"
        ),
        Node(
            package="tank_pressure_demo_cpp",
            executable="compressor_service",
            name="compressor_service",
            output="screen"
        ),
        Node(
            package="tank_pressure_demo_cpp",
            executable="fill_action_server",
            name="fill_action_server",
            output="screen"
        ),
        # Optional: bring up the scheduler too
        Node(
            package="tank_pressure_demo_cpp",
            executable="scheduler_node",
            name="scheduler_node",
            output="screen"
        ),
        # Optional: run a one-shot action client (set target here)
        # Node(
        #     package="tank_pressure_demo_cpp",
        #     executable="fill_action_client",
        #     name="fill_action_client",
        #     arguments=["705"],
        #     output="screen"
        # ),
    ])

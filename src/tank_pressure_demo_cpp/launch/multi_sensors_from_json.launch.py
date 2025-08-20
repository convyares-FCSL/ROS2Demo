from launch import LaunchDescription
from launch.actions import OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import json

PACKAGE    = 'tank_pressure_demo_cpp'
EXECUTABLE = 'pressure_publisher'  # we’re targeting this

def _load_json_config(json_path: str):
    return json.loads(Path(json_path).read_text())

def _nodes_from_json(json_path):
    cfg = _load_json_config(json_path)
    groups = []
    for item in cfg:
        ns     = item.get('ns', '')
        params = item.get('params', {})
        groups.append(
            GroupAction([
                PushRosNamespace(ns),
                Node(
                    package=PACKAGE,
                    executable=EXECUTABLE,
                    name='pressure_publisher',
                    output='screen',
                    parameters=[params],
                )
            ])
        )
    return groups

def _launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory(PACKAGE))
    json_path = str(pkg_share / 'config' / 'transducers.json')
    return _nodes_from_json(json_path)

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])

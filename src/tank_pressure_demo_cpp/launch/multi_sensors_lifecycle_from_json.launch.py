from launch import LaunchDescription
from launch.actions import OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import json

PKG='tank_pressure_demo_cpp'
EXEC='pressure_publisher_lc'

def nodes_from_json(json_path):
    cfg = json.loads(Path(json_path).read_text())
    groups=[]
    for item in cfg:
        ns=item['ns']; params=item['params']
        groups.append(GroupAction([
            PushRosNamespace(ns),
            Node(package=PKG, executable=EXEC, name='pressure_publisher_lc',
                 output='screen', parameters=[params])
        ]))
    return groups

def setup(context,*args,**kw):
    json_path = str(Path(get_package_share_directory(PKG)) / 'config' / 'transducers.json')
    return nodes_from_json(json_path)

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])

# ROS 2 Tank Pressure Demo (Actions)

This repo demonstrates a ROS 2 **Action** workflow for a “fill to target pressure” process, with C++ nodes and small Python utilities.

## Packages
- `hyfleet_interfaces` — custom action: `FillToTarget`
- `tank_pressure_demo_cpp` — C++ action **server** and **client**
- `tank_pressure_tools_py` — Python **logger** and **plotter**

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Run (launch server + logger + client)
```bash
ros2 launch tank_pressure_demo_cpp action_demo.launch.py
```

The launch starts:
- `fill_action_server` with params (defaults shown in `launch/action_demo.launch.py`)
- `pressure_logger` writing `/tmp/tank_pressure.csv`
- `fill_action_client` sending a goal (default target 705 bar)

### Parameters (server)
- `start_bar` (double) — initial tank pressure (bar)
- `ramp_bar_per_s` (double) — ramp rate (bar/s)
- `aprr_mpa_per_min` (double) — APRR in MPa/min; if >0, overrides `ramp_bar_per_s`
- `tick_ms` (int) — control loop period (ms)

Set at runtime:
```bash
ros2 param set /fill_action_server ramp_bar_per_s 1.5
ros2 param set /fill_action_server tick_ms 50
```

### Manual run (without launch)
```bash
# Terminal A — server
ros2 run tank_pressure_demo_cpp fill_action_server --ros-args   -p start_bar:=96.0 -p aprr_mpa_per_min:=1.5 -p tick_ms:=100

# Terminal B — client
ros2 run tank_pressure_demo_cpp fill_action_client 705.0
```

### Telemetry
- Topic: `/tank_pressure` (`std_msgs/Float64`)
- CSV: `/tmp/tank_pressure.csv` via `pressure_logger`
- Plot: use PlotJuggler (ROS2 Streaming → subscribe to `/tank_pressure`)

### Development notes
- Control path is C++ (actions/timer). Tooling (logging/plotting) is Python for speed.
- Server enforces single active goal; supports feedback + (soon) cancel/abort.

## Legacy demo (publisher/service)
See `README_pubsub.md` for the older pub/sub/service nodes.

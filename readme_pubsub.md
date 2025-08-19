# Legacy: Publisher / Subscriber / Compressor Service

This older demo predates the action-based flow and includes:
- `pressure_publisher` — publishes `std_msgs/Float64` pressure
- `pressure_subscriber` — logs pressure
- `compressor_service` — simple on/off service
- `scheduler_node` — (legacy) talks directly to compressor; not used now

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select tank_pressure_demo_cpp
source install/setup.bash
```

## Run
```bash
ros2 run tank_pressure_demo_cpp pressure_publisher
ros2 run tank_pressure_demo_cpp pressure_subscriber
ros2 run tank_pressure_demo_cpp compressor_service
```

> ⚠️ The new action demo supersedes this. Keep for reference only.

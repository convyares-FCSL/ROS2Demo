# Tank Pressure Demo (C++)

This ROS 2 package provides a simple demo simulating tank pressure publishing and compressor control using publishers, parameters, and services.

## Build Instructions

```bash
docker exec -it rosdev bash

cd ~/ros2_ws
colcon build --packages-select tank_pressure_demo_cpp --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Run Instructions

```bash
docker exec -it rosdev sh
cd opt/ros/jazzy

source /opt/ros/jazzy/setup.bash
```

### Pressure Publisher

```bash
ros2 run tank_pressure_demo_cpp pressure_publisher
```

### Pressure Subscriber

```bash
ros2 run tank_pressure_demo_cpp pressure_subscriber
```

### Compressor Service

```bash
ros2 run tank_pressure_demo_cpp compressor_service
```

---

## Parameters

The **publisher** supports runtime-configurable parameters:

- **bank\_id** *(string)*: Which bank is being simulated (`BankA`, `BankB`, `BankC`).
- **start\_bar** *(double)*: Starting tank pressure (bar).
- **jitter\_bar** *(double)*: Random jitter added to the pressure value. Must be `>= 0`.
- **period\_ms** *(int)*: Publish period in milliseconds. Must be `> 0`.

### Inspect Parameters

```bash
ros2 param list /tank_pressure_pub_cpp
ros2 param get /tank_pressure_pub_cpp bank_id
```

### Update Parameters at Runtime

```bash
ros2 param set /tank_pressure_pub_cpp bank_id BankB
ros2 param set /tank_pressure_pub_cpp jitter_bar 5.0
ros2 param set /tank_pressure_pub_cpp period_ms 1000
```

---

## Services

The **compressor service** provides three ROS 2 services:

- **/start\_compressor** (`std_srvs/srv/SetBool`)\
  Start (`data: true`) or stop (`data: false`) the compressor.

- **/fault\_interlock** (`std_srvs/srv/Trigger`)\
  Force the interlock into a faulted state. Prevents compressor state changes until reset.

- **/reset\_interlock** (`std_srvs/srv/Trigger`)\
  Reset the interlock fault to allow compressor operation again.

### Example Calls

Start the compressor:

```bash
ros2 service call /start_compressor std_srvs/srv/SetBool "{data: true}"
```

Stop the compressor:

```bash
ros2 service call /start_compressor std_srvs/srv/SetBool "{data: false}"
```

Trigger an interlock fault:

```bash
ros2 service call /fault_interlock std_srvs/srv/Trigger "{}"
```

Reset the interlock fault:

```bash
ros2 service call /reset_interlock std_srvs/srv/Trigger "{}"
```

Check available services:

```bash
ros2 service list | grep compressor
```




# Setup ros workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

```

# Trigger calibration (voor de talker_movella_dot in vr_bridge package)
``` bash
ros2 topic pub --once calibrate std_msgs/msg/Empty "{}"
```

# Change latency parameter (voor de arduino_parallel_v2)
```bash
ros2 run vr_telepresence arduino_parallel_v2 --ros-args -p latency_ms:=200.0
ros2 param set /arduino_parallel_v2 latency_ms 200.0
```
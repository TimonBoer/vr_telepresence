```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

    # Trigger calibration
ros2 topic pub --once calibrate std_msgs/msg/Empty "{}"
```
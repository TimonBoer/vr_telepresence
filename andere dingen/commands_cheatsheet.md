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

# Ip adressen afstellen
''' bash
    #define HMD_TRACKING_IP    "<IP laptop B>"   // Quest stuurt poses naartoe
    #define LAT_COLLECTOR_IP   "<IP laptop B>"   // M2M-records naartoe
    #define VL_SENDER_IP       "<IP laptop A>"   // Quest stuurt video-probes naartoe
    #define VL_COLLECTOR_IP    "<IP laptop A>"   // video-records naartoe

    bij starten arduino_parallel node: export QUEST_IP= IP Quest>
'''
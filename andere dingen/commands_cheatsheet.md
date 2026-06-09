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

Test 1: 0
Test 2: camera latency
Test 3: Arduino latency
Test 4: beide

# Direk
Arduino_parallel_v2
imu_talker.py
video_latency_collector_node
    per test runnen

~./zedsender



# Timon
Python script
- persoonsnr aanpassen
- Opletten op woord nr
Stoppen bij 8, 13, 18, 23

quest_bridge_node

latency_collector node
    per test runnen

```bash
# eerst calibreren
ros2 topic pub --once calibrate std_msgs/msg/Empty "{}"
# even wachten tot nek is gedraaid
# dan imu calibreren
ros2 topic pub --once imu/calibrate std_msgs/msg/Empty "{}"

#zet recording aan voor elke test
ros2 bag record quest/orientation imu/orientation



ros2 bag info _
```
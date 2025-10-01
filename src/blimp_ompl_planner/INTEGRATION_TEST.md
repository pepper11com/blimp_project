# 🚁 OMPL + Blimp Integration Test Guide

## 🎯 **Complete Workflow Test**

### **Terminal Setup:**

**Terminal 1: SLAM/Odometry**
```bash
cd ~/blimp_ws && source install/setup.bash
ros2 launch blimp_core blimp_localization_slam_launch.py
```

**Terminal 2: OMPL Planner**
```bash
cd ~/blimp_ws && source install/setup.bash
ros2 launch blimp_ompl_planner ompl_planner_launch.py
```

**Terminal 3: Blimp Controller**
```bash
cd ~/blimp_ws && source install/setup.bash
# Enable MSP hardware output; adjust serial_device if needed
ros2 run blimp_navigation blimp_navigator_node \
   --ros-args \
      --params-file ~/blimp_ws/src/blimp_navigation/param/blimp_navigator.yaml \
      -p enable_actuators:=true \
      -p serial_device:=/dev/ttyAMA0
```

> ⚠️ Actuators stay idle unless `enable_actuators` is set to `true`. Leave it `false` when bench-testing without the flight controller connected. The default parameter file caps thrust at ±20% and the console prints a “Navigator Status” block each second so you can verify the applied vs. requested commands.

> 🔁 On startup the navigator briefly drives each motor forward and reverse (default ~4 s each) and sweeps both servos. Keep the props clear and use the sequence to confirm wiring. To skip it, pass `-p actuator_startup_check:=false` or adjust the `actuator_check_*` parameters.

**Terminal 4: Navigation Commands**
```bash
cd ~/blimp_ws && source install/setup.bash

# Set flying altitude
ros2 run blimp_ompl_planner set_altitude.py 1.5

# Send navigation goal
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: -3.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## 🔄 **What Should Happen:**

1. **Goal Handler** receives `/goal_pose`
2. **Goal Handler** sends `NavigateToPose` action to **OMPL Planner**
3. **OMPL Planner** creates 3D collision-free path
4. **OMPL Planner** publishes path to `/blimp/planned_path`
5. **Blimp Navigator** receives path and follows waypoints
6. **Blimp Controller** sends motor/servo commands
7. **Blimp** flies along the safe 3D path! 🎉

## 📊 **Topics to Monitor:**

```bash
# Goal commands
ros2 topic echo /goal_pose

# OMPL planned path
ros2 topic echo /blimp/planned_path

# Navigation status
ros2 topic echo /blimp_status

# Action feedback
ros2 action list
ros2 action info /navigate_to_pose
```

## 🐛 **Troubleshooting:**

### **Problem: No path received**
```bash
# Check OMPL planner is running
ros2 node list | grep ompl

# Check if action server is active
ros2 action list | grep navigate_to_pose

# Check collision service
ros2 service list | grep collision
```

### **Problem: Path not followed**
```bash
# Check if navigator received path
ros2 topic echo /blimp/planned_path --once

# Check navigator status
ros2 topic echo /blimp_status
```

### **Problem: No motor movement**
- Check serial connection to flight controller
- Verify `/dev/ttyAMA0` permissions
- Check debug output for PID values

## 🎮 **Foxglove Integration:**

1. **Set Altitude Panel:**
   - Topic: `/set_altitude`
   - Type: `std_msgs/Float32`
   - Message: `{"data": 1.5}`

2. **Goal Panel:**
   - Topic: `/goal_pose`
   - Type: `geometry_msgs/PoseStamped`
   - Message: `{"header": {"frame_id": "map"}, "pose": {"position": {"x": 2.0, "y": -3.0, "z": 0.0}, "orientation": {"w": 1.0}}}`

3. **Visualize:**
   - `/blimp/planned_path` (Path)
   - `/octomap_binary` (OctoMap)
   - `/tf` (Robot position)

## 🚀 **Expected Behavior:**

✅ **Path Planning**: OMPL creates smooth 3D paths avoiding obstacles  
✅ **3D Path Tracking**: Magnetic attraction to planned route (not just waypoints!)  
✅ **Altitude Correction**: Auto-corrects when moved away from path vertically  
✅ **Obstacle Avoidance**: Built-in collision detection and avoidance  
✅ **Adaptive Control**: Slows down for altitude corrections, prioritizes 3D tracking  
✅ **Heading Control**: Always turns toward closest path point  
✅ **Self-Recovery**: Returns to path when manually displaced in any direction  

## 🧲 **3D Path Tracking Features:**

- **Magnetic Pull**: Blimp is "attracted" to the planned 3D route
- **Altitude Recovery**: If moved down 1m, servos automatically move up to rejoin path  
- **Smart Targeting**: Finds closest point on path, then aims slightly ahead
- **Speed Modulation**: Slows horizontal motion during large altitude corrections
- **Visual Feedback**: Status shows "ALTITUDE CORRECTION" when actively climbing/descending

## 🚀 **Reversible Motor Capabilities (NEW!):**

- **Counter-Rotation**: Large heading errors use opposing thrust (L=+10%, R=-10%)
- **Reverse Thrust**: Target behind? Reverse instead of slow turn-around!
- **Active Braking**: Close to waypoint? Reverse thrust for precise stopping
- **Obstacle Escape**: Hit obstacle? Reverse away while turning to escape
- **Vectored Control**: Independent forward/reverse + differential steering
- **Smart Movement**: Automatically chooses fastest approach (forward vs reverse)

### **🎮 Motor Control Mapping:**
```
Motor Output:  -100%    0%    +100%
PWM Signal:    1000µs  1500µs  2000µs  
Movement:      REVERSE NEUTRAL FORWARD
```

The blimp now has **full 6-DOF control** with intelligent movement strategies! 🚁⚡
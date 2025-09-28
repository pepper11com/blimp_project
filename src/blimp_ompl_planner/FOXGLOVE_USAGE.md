# 🦊 Foxglove + OMPL Planner Integration Guide

## 📍 **How to Send Navigation Goals**

### **Method 1: Set Altitude + 2D Goals** ⭐ **RECOMMENDED**
Perfect for Foxglove! Set altitude once, then send XY goals:

```bash
# 1. Set your flying altitude (stays persistent until changed)
ros2 run blimp_ompl_planner set_altitude.py 2.0

# 2. Send XY goals (Z will automatically use your altitude setting)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: -6.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### **Method 2: Direct 3D Topic Publishing**
Send goals directly with full 3D coordinates:

```bash
# Basic 3D goal (x, y, z in meters)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: -3.0, z: 1.5},
    orientation: {w: 1.0}
  }
}'
```

### **Method 3: Using Foxglove Publish Panel** 🎯
Perfect workflow for 3D navigation:

**Step 1: Set Altitude (do this once)**
- Add **Publish** panel in Foxglove
- Topic: `/set_altitude`
- Type: `std_msgs/Float32`
- Message: `{"data": 2.0}`  ← Your desired height
- Click Publish

**Step 2: Send XY Goals (as many as you want)**
- Add another **Publish** panel
- Topic: `/goal_pose`
- Type: `geometry_msgs/PoseStamped`
- Message template (just change X,Y):
```json
{
  "header": {"frame_id": "map"},
  "pose": {
    "position": {"x": 2.0, "y": -3.0, "z": 0.0},
    "orientation": {"w": 1.0}
  }
}
```
**Note**: Z value is ignored - your altitude setting is used automatically!

### **Method 3: Interactive Script** 🚀
```bash
# Easy interactive goal setting
cd ~/blimp_ws && source install/setup.bash
python3 src/blimp_ompl_planner/scripts/send_navigation_goal.py 2.0 -3.0 1.5
```

## 🔍 **Foxglove Visualization Setup**

### **Topics to Subscribe to:**
- **OctoMap**: `/octomap_binary` (type: `octomap_msgs/Octomap`)
- **Full Path**: `/blimp/planned_path_full` (type: `nav_msgs/Path`) - Complete route
- **Remaining Path**: `/blimp/planned_path_remaining` (type: `nav_msgs/Path`) - Shrinks as waypoints completed ⭐
- **Current OMPL Path**: `/blimp/planned_path` (type: `nav_msgs/Path`) - Raw OMPL output
- **Robot Position**: `/tf` and `/tf_static` (type: `tf2_msgs/TFMessage`)
- **Point Cloud**: `/rtabmap/cloud_map` (type: `sensor_msgs/PointCloud2`)
- **Goal Pose**: `/goal_pose` (type: `geometry_msgs/PoseStamped`)

### **Recommended Panel Layout:**
1. **3D Panel**: Main visualization
   - Add OctoMap (set transparency ~0.7)
   - Add **Full Path** (blue) - `/blimp/planned_path_full` 
   - Add **Remaining Path** (green) - `/blimp/planned_path_remaining` ⭐ 
   - Add TF tree (show robot frames)
   - Add Point Cloud (if available)

2. **Publish Panel**: For sending goals
   - Topic: `/goal_pose`
   - Quick goal templates

3. **Topic List**: Monitor active topics

4. **Diagnostics**: Check system health

## ⚡ **Quick Test Commands**

```bash
# 1. Start the navigation stack
cd ~/blimp_ws && source install/setup.bash
ros2 launch blimp_ompl_planner blimp_3d_navigation_launch.py

# 2. In another terminal, send a test goal
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: -6.0, z: 1.0}, orientation: {w: 1.0}}}'

# 3. Check if goal was received
ros2 topic echo /navigate_to_pose/goal --once
```

## 🎯 **Goal Coordinate Tips**

- **Frame**: Always use `"map"` frame
- **X/Y**: Horizontal position in meters
- **Z**: Altitude in meters (0.5 to 5.0 recommended)
- **Orientation**: Use `{w: 1.0}` for default (facing forward)

## 🔧 **Troubleshooting**

```bash
# Check if planner is running
ros2 node list | grep planner

# Monitor path topics
ros2 topic echo /blimp/planned_path_full --once
ros2 topic echo /blimp/planned_path_remaining --once

# Check path lengths (should shrink over time)
ros2 topic hz /blimp/planned_path_remaining

# Monitor navigation action
ros2 action list | grep navigate

# Check TF tree
ros2 run tf2_tools view_frames.py
```
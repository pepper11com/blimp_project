# OMPL Planner Utility Scripts

This directory contains utility scripts for testing and managing the 3D OMPL planner.

## 🛠️ Available Scripts

### **`send_navigation_goal.py`** ⭐ **ESSENTIAL**
Send navigation goals to the OMPL planner for testing.

**Usage:**
```bash
python3 send_navigation_goal.py <x> <y> <z> [yaw]
```

**Examples:**
```bash
# Send goal to position (2.0, 1.0, 1.5) meters
python3 send_navigation_goal.py 2.0 1.0 1.5

# Send goal with specific orientation
python3 send_navigation_goal.py 2.0 1.0 1.5 1.57  # 90 degrees yaw
```

### **`quick_bounds_analyzer.py`** 🔍 **CONFIGURATION**
Analyze your OctoMap to automatically determine optimal planning bounds.

**Usage:**
```bash
python3 quick_bounds_analyzer.py
```

**What it does:**
- Analyzes existing OctoMap data
- Suggests optimal planning bounds for your room
- Automatically updates the config file
- Should be run once when setting up in a new environment

### **`path_visualizer.py`** 👀 **VISUALIZATION**
Keeps planned paths visible in RViz by republishing them continuously.

**Usage:**
```bash
python3 path_visualizer.py &  # Run in background
```

**What it does:**
- Subscribes to planned paths
- Republishes them every second to keep them visible in RViz
- Shows path statistics (length, waypoint count)

### **`test_collision_radius.py`** 🧪 **TESTING**
Generate test scenarios to verify collision detection with different robot radii.

**Usage:**
```bash
python3 test_collision_radius.py
```

**What it does:**
- Analyzes current robot position
- Suggests test goals at various distances
- Helps verify that robot_radius parameter is working correctly

## 🚀 Quick Start

1. **Set up your environment:**
   ```bash
   python3 quick_bounds_analyzer.py
   ```

2. **Start the planner:**
   ```bash
   ros2 launch blimp_ompl_planner ompl_planner_launch.py
   ```

3. **Send a test goal:**
   ```bash
   python3 send_navigation_goal.py 2.0 0.0 1.0
   ```

4. **Visualize paths (optional):**
   ```bash
   python3 path_visualizer.py &
   ```
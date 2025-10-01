# Dynamic Ceiling Detection for OMPL Planner

## Overview
This feature prevents the blimp from planning paths that would hit unmapped ceilings. Since black surfaces (like dark ceilings and floors) are often not visible to RGB-D cameras, they don't appear in the OctoMap. This creates an invisible hazard where the planner might generate paths that fly into the ceiling.

## Solution
The OctomapManager now dynamically computes the maximum Z coordinate of all **occupied** voxels in the map. This represents the highest visible obstacle. The planner then constrains its search space to stay below this height, with a configurable safety margin.

## How It Works

### 1. Maximum Occupied Voxel Scan
During each OctoMap update (`updateBoundsLocked()`), the system:
- Iterates through all leaf nodes in the octree
- Tracks the maximum Z coordinate of occupied voxels
- Applies a safety margin below this height

### 2. Adaptive Bounds
The planning bounds `max_z` is set to the **minimum** of:
- Initial bounds from config (`bounds_max_z`)
- Map bounding box max Z + padding
- Highest occupied voxel Z - safety margin

This ensures the planner never generates paths above visible obstacles.

### 3. Safety Margin
Configurable parameter `safety_margin_from_ceiling` (default: 0.3m) creates a buffer zone below the highest obstacle. This accounts for:
- Voxel quantization errors
- Blimp size and dynamics
- Sensor noise and drift

## Configuration

### Parameter File
`blimp_ompl_planner/config/blimp_planning_bounds.yaml`:
```yaml
ompl_planner_node:
  ros__parameters:
    # Safety parameters
    safety_margin_from_ceiling: 0.3  # Distance (meters) to stay below highest obstacle
```

### Adjusting Safety Margin
- **Increase** (e.g., 0.5m) for:
  - Larger safety buffer
  - Faster/aggressive flight
  - Uncertain localization
  
- **Decrease** (e.g., 0.2m) for:
  - Tighter paths in confined spaces
  - More vertical freedom
  - Very accurate localization

## Logging
The system logs ceiling constraint activity:
```
[ompl_planner_node]: Ceiling constraint active: highest obstacle at 2.45m, limiting planning to 2.15m
```

This message appears when:
- An occupied voxel ceiling is detected
- The ceiling constraint is more restrictive than configured bounds
- Throttled to once per 5 seconds to avoid spam

## Benefits

1. **Safety**: Prevents collisions with unmapped ceilings
2. **Automatic**: No manual ceiling measurement needed
3. **Adaptive**: Adjusts as the map grows and explores new areas
4. **Robust**: Works even when ceiling is completely invisible to sensors

## Technical Details

### Modified Files
- `blimp_ompl_planner/include/blimp_ompl_planner/octomap_manager.hpp`
  - Added `safety_margin_from_ceiling_` member
  - Added `setSafetyMarginFromCeiling()` method
  - Modified `updateBoundsLocked()` to return ceiling info

- `blimp_ompl_planner/src/octomap_manager.cpp`
  - Implemented voxel scanning for max Z computation
  - Added ceiling constraint logic in `updateBoundsLocked()`
  - Added informative logging

- `blimp_ompl_planner/src/ompl_planner_node.cpp`
  - Added parameter loading for `safety_margin_from_ceiling`
  - Calls `setSafetyMarginFromCeiling()` during initialization

- `blimp_ompl_planner/config/blimp_planning_bounds.yaml`
  - Added `safety_margin_from_ceiling` parameter with documentation

### Algorithm Complexity
- **Time**: O(n) where n = number of leaf nodes in octree
- **Space**: O(1) additional memory
- **Frequency**: Runs once per OctoMap update (~1-5 Hz typical)

## Example Scenario

**Before (without ceiling detection):**
```
Room height: 3.0m (black ceiling, unmapped)
Highest visible obstacle: 2.5m (tall furniture)
Configured max_z: 5.0m

Planner generates path at 2.8m → COLLISION with invisible ceiling
```

**After (with ceiling detection):**
```
Room height: 3.0m (black ceiling, unmapped)
Highest visible obstacle: 2.5m (tall furniture)
Safety margin: 0.3m
Configured max_z: 5.0m

Computed ceiling constraint: 2.5 - 0.3 = 2.2m
Planner limited to 2.2m → SAFE paths only
```

## Future Enhancements

Potential improvements:
1. **Regional ceiling heights**: Track max Z per XY region for sloped/varied ceilings
2. **Temporal filtering**: Smooth ceiling estimates over multiple updates
3. **Confidence metrics**: Weight recent vs. old ceiling measurements
4. **Floor detection**: Similar logic for minimum Z bounds

## Testing Recommendations

1. **Visual Verification**: 
   - Check RViz bounds markers stay below ceiling
   - Monitor log messages during flight

2. **Deliberate Test**:
   - Create environment with tall object near ceiling
   - Request goal above the object
   - Verify planner generates path below object + margin

3. **Edge Cases**:
   - Empty map (no occupied voxels) → uses configured bounds
   - Very low ceiling → planner may fail if too restrictive
   - Gradual ceiling lowering → bounds adapt smoothly

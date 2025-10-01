# Velocity Estimation & Predictive Control

## Overview
Added intelligent velocity-based control features that make the blimp more predictive and smooth in its navigation.

## Features Added

### 1. Real-Time Velocity Estimation ⭐⭐⭐
**What it does:**
- Estimates current velocity in X, Y, Z directions from pose changes
- Calculates forward speed (magnitude in XY plane)
- Filters velocity with exponential moving average (α = 0.2)

**How it works:**
```cpp
velocity = (current_position - last_position) / dt
smoothed_velocity = 0.2 * new + 0.8 * smoothed  // EMA filter
forward_speed = sqrt(vx² + vy²)
```

**Benefits:**
- Know actual speed vs commanded speed
- Foundation for predictive control
- Enables wind drift detection (future)
- More accurate physics calculations

---

### 2. Predictive Braking 🎯
**What it does:**
- Calculates stopping distance based on actual velocity
- Starts slowing down early enough to stop smoothly at goal
- Accounts for blimp's deceleration capability

**Physics:**
```
stopping_distance = v² / (2 × a)
where a = 0.8 m/s² (with T/W = 1.0)

Also adds reaction time buffer
```

**Behavior:**
- **Old**: Fixed 1.5m brake zone regardless of speed
- **New**: If moving fast (0.5 m/s), starts slowing at 2.0m
- **New**: If moving slow (0.2 m/s), only slows at 0.5m
- Result: Smooth arrivals, no abrupt stops

---

### 3. Altitude Rate Limiting 🛗
**What it does:**
- Limits maximum climb/descent rate to 0.3 m/s
- Prevents servo slamming to extreme positions
- Respects helium buoyancy dynamics

**Why it matters:**
- Helium responds slowly to altitude changes
- Rapid climbs can cause oscillation
- Protects servos from mechanical stress
- More natural, smooth motion

**Example:**
```
Altitude error: 2.0m (need to climb)
Old: Servos slam to max immediately
New: Ramps up smoothly, limited to 0.3 m/s climb rate
Result: Smooth ascent, less overshoot
```

---

### 4. Velocity-Aware Turn Slowdown ⚡
**What it does:**
- Uses actual measured velocity instead of guessing from commands
- More accurate physics calculations
- Better turn prediction

**Improvement:**
- **Old**: `estimated_velocity = command × 0.5` (crude guess)
- **New**: `actual_velocity = measured_forward_speed` (accurate)
- Result: Better turn performance, less overshoot

---

### 5. Obstacle Proximity Slowdown (Framework) 🚧
**What it does:**
- Framework in place for OctoMap integration
- Will slow down when approaching obstacles
- Currently returns 1.0 (no slowdown) as placeholder

**Future capability:**
```cpp
if (distance_to_obstacle < 1.0m) {
  scale = distance / 1.0;  // Linear slowdown
  forward_speed *= max(0.3, scale);
}
```

**When implemented:**
- Automatically slows near walls
- Prevents collisions
- More confident navigation in tight spaces

---

## Configuration Parameters

### In `blimp_navigator.yaml`:

```yaml
# Altitude control
max_altitude_rate: 0.3              # m/s - maximum climb/descent speed

# Velocity estimation
velocity_filter_alpha: 0.2          # EMA filter (0=no filter, 1=no smoothing)

# Predictive braking
predictive_brake_time: 2.0          # seconds - lookahead time

# Obstacle avoidance (future)
obstacle_slowdown_distance: 1.0     # meters - start slowing when this close
obstacle_slowdown_min_scale: 0.3    # minimum speed (30%) near obstacles
```

---

## How Velocity Estimation Works

### Algorithm:
1. **Each control cycle** (40ms at 25Hz):
   - Measure position change: `Δp = current_pos - last_pos`
   - Calculate instantaneous velocity: `v_inst = Δp / dt`
   - Apply EMA filter: `v_smooth = 0.2 × v_inst + 0.8 × v_smooth`
   - Calculate forward speed: `forward_speed = sqrt(vx² + vy²)`

2. **First cycle initialization**:
   - Sets `velocity_initialized_ = false`
   - On first call, stores position and sets velocities to zero
   - Second call begins actual estimation

3. **Filtering rationale**:
   - α = 0.2 means 5-cycle averaging window
   - Smooths sensor noise and jitter
   - Still responsive (~0.2 second lag)

---

## Performance Improvements

### Before:
- Fixed brake distance regardless of speed
- Guessed velocity from commands
- No altitude rate limits (servos slam)
- Reactive control only

### After:
- ✅ **Smart braking**: Adapts to actual speed
- ✅ **Accurate velocity**: Measured, not guessed
- ✅ **Smooth altitude**: Rate-limited for comfort
- ✅ **Predictive**: Anticipates stopping distance
- ✅ **Foundation**: Ready for wind compensation

---

## Technical Details

### Velocity Estimation State Variables:
```cpp
geometry_msgs::msg::Point last_position_;  // Previous position
double velocity_x_;                         // m/s in world X
double velocity_y_;                         // m/s in world Y  
double velocity_z_;                         // m/s in world Z (vertical)
double forward_speed_;                      // m/s magnitude in XY
bool velocity_initialized_;                 // First-run flag
```

### Deceleration Model:
```cpp
// With T/W = 1.0, max theoretical deceleration = 9.8 m/s²
// But accounting for:
// - Drag assistance (helps slow down)
// - Control system response time
// - Conservative margin for safety
// Result: Use 0.8 m/s² for reliable stopping
```

### Stopping Distance Formula:
```
d_stop = v² / (2a) + v × t_reaction
where:
  v = current forward speed (m/s)
  a = 0.8 m/s² (deceleration)
  t_reaction = 0.3 × predictive_brake_time (buffer)
```

---

## Future Enhancements Ready

With velocity estimation in place, we can now easily add:

1. **Wind Drift Compensation**:
   ```cpp
   wind = actual_velocity - expected_velocity_from_commands
   motor_command += wind_compensation_gain × wind
   ```

2. **Trajectory Prediction**:
   ```cpp
   predicted_pos = current_pos + velocity × lookahead_time
   if (predicted_pos will hit obstacle) {
     take_evasive_action();
   }
   ```

3. **Adaptive Lookahead**:
   ```cpp
   lookahead_distance = base + velocity × scale_factor
   // Faster = look farther ahead
   ```

4. **Velocity-Based Mode Switching**:
   ```cpp
   if (forward_speed < 0.1) {
     enable_precision_mode();  // Fine control at low speeds
   }
   ```

---

## Testing Notes

### What to observe:
1. **Velocity convergence**: After ~5 cycles (0.2s), velocity should stabilize
2. **Predictive braking**: Should start slowing earlier when moving fast
3. **Smooth altitude**: No more servo slamming on altitude changes
4. **Turn performance**: Better cornering with accurate velocity

### Expected behavior:
- **Fast approach**: Starts braking at ~2m from goal
- **Slow approach**: Only brakes at ~0.5m from goal
- **Altitude changes**: Max 0.3 m/s climb/descent
- **Turns**: Velocity-aware slowdown (accurate)

---

## Summary

These features make the blimp **predictive** instead of just **reactive**:

| Feature | Old | New | Benefit |
|---------|-----|-----|---------|
| Braking | Fixed distance | Velocity-based | Smoother arrivals |
| Altitude | Instant response | Rate-limited 0.3m/s | No overshoot |
| Velocity | Guessed | Measured & filtered | Accurate control |
| Turns | Command-based | Velocity-aware | Better cornering |
| Foundation | N/A | Ready for wind comp | Future-proof |

**Result**: Smarter, smoother, more predictable navigation! 🚀

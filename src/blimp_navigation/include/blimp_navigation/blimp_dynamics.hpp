#ifndef BLIMP_NAVIGATION__BLIMP_DYNAMICS_HPP_
#define BLIMP_NAVIGATION__BLIMP_DYNAMICS_HPP_

#include <cmath>

namespace blimp_navigation
{

/**
 * @brief Physical model and dynamics predictor for blimp motion
 * 
 * Based on actual blimp specifications:
 * - Volume: 2.35 m³
 * - Length: 3.06m, Diameter: 0.88m
 * - Mass: ~1.0 kg (neutrally buoyant)
 * - Surface area: 6.92 m²
 * - Motors: 2x 4300KV with Gemfan 1636 40mm props
 */
struct BlimpDynamics
{
  // Physical constants (from blimp specs)
  static constexpr double BLIMP_LENGTH = 3.06;        // meters
  static constexpr double BLIMP_DIAMETER = 0.88;      // meters
  static constexpr double BLIMP_MASS = 1.0;           // kg (approx with payload)
  static constexpr double SURFACE_AREA = 6.92;        // m²
  
  // Estimated aerodynamic coefficients (typical for blimps)
  static constexpr double DRAG_COEF_AXIAL = 0.05;     // Drag along length
  static constexpr double DRAG_COEF_LATERAL = 0.8;    // Drag perpendicular to length
  static constexpr double AIR_DENSITY = 1.225;        // kg/m³ at sea level
  
  // Motor/thrust characteristics (measured: 4300KV + Gemfan 1636 on 2S 7.4V)
  static constexpr double MAX_THRUST_PER_MOTOR = 0.5;  // Newtons (0.42-0.63N range, using middle)
  static constexpr double TOTAL_MAX_THRUST = 1.0;      // Newtons (both motors)
  static constexpr double THRUST_TO_WEIGHT = 1.0;      // Ratio (excellent for blimp!)
  static constexpr double THRUST_TO_NORM = 0.20;       // max motor norm command
  
  /**
   * @brief Estimate drag force opposing forward motion
   * @param velocity Forward velocity in m/s
   * @return Drag force in Newtons
   */
  static double estimateAxialDrag(double velocity)
  {
    // F_drag = 0.5 * rho * Cd * A * v²
    // Use frontal area (circle with diameter 0.88m)
    const double frontal_area = M_PI * (BLIMP_DIAMETER / 2.0) * (BLIMP_DIAMETER / 2.0);
    return 0.5 * AIR_DENSITY * DRAG_COEF_AXIAL * frontal_area * velocity * velocity;
  }
  
  /**
   * @brief Estimate lateral drag during turns
   * @param lateral_velocity Velocity perpendicular to blimp axis in m/s
   * @return Lateral drag force in Newtons
   */
  static double estimateLateralDrag(double lateral_velocity)
  {
    // Much higher drag when moving sideways (large projected area)
    const double lateral_area = BLIMP_LENGTH * BLIMP_DIAMETER;
    return 0.5 * AIR_DENSITY * DRAG_COEF_LATERAL * lateral_area * 
           lateral_velocity * std::abs(lateral_velocity);
  }
  
  /**
   * @brief Estimate minimum turning radius at given forward speed
   * @param forward_velocity Forward speed in m/s
   * @param max_yaw_rate Maximum yaw rate in rad/s (from differential thrust)
   * @return Minimum turning radius in meters
   * 
   * Note: Motors are center-mounted (basket at middle of blimp),
   * giving excellent pivot capability with minimal turn radius.
   */
  static double estimateTurningRadius(double forward_velocity, double max_yaw_rate = 0.5)
  {
    if (max_yaw_rate < 1e-6) return 1e6; // Essentially infinite
    // R = v / omega
    // With center-mounted motors, can achieve higher yaw rates (0.5 rad/s vs 0.3)
    return forward_velocity / max_yaw_rate;
  }
  
  /**
   * @brief Predict how long it takes to reach target speed
   * @param current_speed Current speed in m/s
   * @param target_speed Desired speed in m/s
   * @return Time to reach 90% of target speed in seconds
   */
  static double estimateAccelerationTime(double current_speed, double target_speed)
  {
    const double speed_diff = std::abs(target_speed - current_speed);
    // With T/W ratio of 1.0 and 1kg mass, acceleration is excellent!
    // Maximum acceleration: F/m = 1.0N / 1.0kg = 1.0 m/s²
    // But limited by drag and control smoothness
    // Time constant tau ≈ m / b where b is drag coefficient
    // With high thrust, settling is fast: tau ≈ 0.5-1.0 seconds
    const double time_constant = 0.7; // seconds (responsive!)
    return time_constant * 2.3; // 2.3 * tau for 90% settling (~1.6 seconds)
  }
  
  /**
   * @brief Calculate feed-forward thrust to maintain constant speed against drag
   * @param desired_speed Target forward speed in m/s
   * @return Normalized thrust command [0, 1]
   * 
   * Note: With T/W ratio of 1.0, drag is minimal compared to available thrust.
   * This function is mostly for completeness - thrust headroom is excellent.
   */
  static double dragCompensation(double desired_speed)
  {
    const double drag_force = estimateAxialDrag(desired_speed);
    const double required_thrust = drag_force; // F_thrust = F_drag for equilibrium
    
    // Convert to normalized command
    // With 1.0N total thrust available, drag is negligible (~0.01-0.05N at cruise)
    double thrust_norm = required_thrust / TOTAL_MAX_THRUST;
    
    // Clamp to reasonable range
    return std::clamp(thrust_norm, 0.0, THRUST_TO_NORM);
  }
  
  /**
   * @brief Calculate slowdown factor for upcoming turn
   * @param heading_error_rad Angular error to target heading in radians
   * @param current_speed Current forward speed in m/s
   * @return Speed scale factor [0.5, 1.0] - lower for tighter turns
   * 
   * Note: With center-mounted motors, turning is much more efficient.
   * The blimp can pivot on its center axis with minimal lateral drift.
   * Less aggressive slowdown needed compared to rear-mounted propulsion.
   */
  static double turnSlowdownFactor(double heading_error_rad, double current_speed)
  {
    const double abs_error = std::abs(heading_error_rad);
    
    // With center-mounted motors, we have excellent turn authority
    // Only need significant slowdown for very large heading errors
    if (abs_error > 2.0 * M_PI / 3.0) { // > 120 degrees (very sharp)
      return 0.5; // Slow to 50% speed for extreme turns
    } else if (abs_error > M_PI / 3.0) { // > 60 degrees
      // Linear interpolation between 60° (scale=0.8) and 120° (scale=0.5)
      const double normalized = (abs_error - M_PI / 3.0) / (2.0 * M_PI / 3.0 - M_PI / 3.0);
      return 1.0 - 0.5 * normalized; // 1.0 -> 0.5
    }
    
    // Moderate errors (<60°), no slowdown needed - excellent turn capability!
    return 1.0;
  }
  
  /**
   * @brief Estimate lateral drift velocity during a turn
   * @param forward_speed Forward speed in m/s  
   * @param yaw_rate Current yaw rate in rad/s
   * @return Estimated lateral drift in m/s
   * 
   * Note: With center-mounted motors, lateral drift is minimal.
   * The blimp rotates around its center of mass, not from the rear.
   */
  static double estimateLateralDrift(double forward_speed, double yaw_rate)
  {
    // With center-mounted propulsion, drift is much less than rear-mounted
    // Mostly from aerodynamic effects at the nose/tail during rotation
    const double centripetal_accel = forward_speed * yaw_rate;
    // Reduced drift coefficient (0.2 vs 0.5 for rear propulsion)
    return 0.2 * BLIMP_LENGTH * yaw_rate;
  }
};

} // namespace blimp_navigation

#endif // BLIMP_NAVIGATION__BLIMP_DYNAMICS_HPP_

#ifndef BLIMP_OMPL_PLANNER__PLANNER_TYPES_HPP_
#define BLIMP_OMPL_PLANNER__PLANNER_TYPES_HPP_

#include <string>

namespace blimp_ompl_planner
{

  struct PlanningBounds
  {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
  };

  struct PathProcessingParameters
  {
    bool enable_simplification;
    bool enable_smoothing;
    int max_simplification_steps;
  };

  struct PlannerParameters
  {
    double robot_radius;
    double planning_timeout;
    std::string planner_id;
    std::string global_frame;
    std::string robot_frame;
    PlanningBounds bounds;
    PathProcessingParameters path_processing;
  };

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__PLANNER_TYPES_HPP_

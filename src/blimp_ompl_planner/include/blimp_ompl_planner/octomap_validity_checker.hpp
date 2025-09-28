#ifndef BLIMP_OMPL_PLANNER__OCTOMAP_VALIDITY_CHECKER_HPP_
#define BLIMP_OMPL_PLANNER__OCTOMAP_VALIDITY_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <octomap/octomap.h>

namespace blimp_ompl_planner
{

    /**
     * @brief Custom State Validity Checker that uses OctoMap for collision detection
     *
     * This class inherits from ompl::base::StateValidityChecker and implements
     * collision checking using an OctoMap. It performs volumetric checking to
     * account for the robot's physical size using a spherical approximation.
     */
    class OctoMapValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        /**
         * @brief Constructor
         *
         * @param si Space information pointer from OMPL
         * @param tree Shared pointer to the OctoMap tree
         * @param robot_radius The radius of the robot for collision checking
         * @param mutex Shared mutex for thread-safe access to the OctoMap
         */
        OctoMapValidityChecker(
            const ompl::base::SpaceInformationPtr &si,
            std::shared_ptr<octomap::OcTree> tree,
            double robot_radius,
            std::shared_ptr<std::mutex> mutex);

        /**
         * @brief Check if a given state is valid (collision-free)
         *
         * @param state Pointer to the OMPL state to check
         * @return true if state is valid, false otherwise
         */
        bool isValid(const ompl::base::State *state) const override;

        /**
         * @brief Update the OctoMap tree reference
         *
         * @param new_tree New OctoMap tree to use for collision checking
         */
        void updateOctoMap(std::shared_ptr<octomap::OcTree> new_tree);

        /**
         * @brief Get the clearance (distance to nearest obstacle) for a state
         *
         * @param state Pointer to the OMPL state
         * @return Distance to nearest obstacle, or max_clearance if no obstacles within range
         */
        double clearance(const ompl::base::State *state) const override;

    private:
        std::shared_ptr<octomap::OcTree> tree_;
        double robot_radius_;
        std::shared_ptr<std::mutex> tree_mutex_;

        // Maximum clearance value to return when no obstacles are nearby
        static constexpr double MAX_CLEARANCE = 10.0;

        /**
         * @brief Check if a point is occupied in the OctoMap
         *
         * @param point 3D point to check
         * @return true if occupied, false if free or unknown
         */
        bool isPointOccupied(const octomap::point3d &point) const;

        /**
         * @brief Get the minimum distance to occupied space around a point
         *
         * @param center Center point to check around
         * @param radius Radius to check within
         * @return Minimum distance to occupied space
         */
        double getMinDistanceToObstacles(const octomap::point3d &center, double radius) const;
    };

} // namespace blimp_ompl_planner

#endif // BLIMP_OMPL_PLANNER__OCTOMAP_VALIDITY_CHECKER_HPP_
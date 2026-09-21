#ifndef __FOLLOW_TRAJECTORY_COMMAND_HPP__
#define __FOLLOW_TRAJECTORY_COMMAND_HPP__

#include <memory>
#include <vector>

#include "driftless/control/trajectory/TrajectoryPoint.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief The namespace for commands which can be sent to the control system
/// @author Matthew Backman
namespace commands {

/// @brief Namespace for commands for the trajectory follower control
/// @author Matthew Backman
namespace trajectory {

struct FollowTrajectoryCommand {
  /// @brief A reference to the robot to follow the trajectory with
  std::shared_ptr<robot::Robot>& m_robot;

  /// @brief The trajectory to follow, as a vector of trajectory points
  std::vector<control::trajectory::TrajectoryPoint>& m_trajectory;
};
}  // namespace trajectory
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
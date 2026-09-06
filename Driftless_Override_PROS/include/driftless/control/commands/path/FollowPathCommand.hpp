#ifndef __FOLLOW_PATH_COMMAND_HPP__
#define __FOLLOW_PATH_COMMAND_HPP__

#include <memory>
#include <vector>

#include "driftless/control/Point.hpp"
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

/// @brief Namespace for commands for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Struct representing a command to start following a path
/// @author Matthew Backman
struct FollowPathCommand {
  /// @brief A reference to the robot to follow the path with
  std::shared_ptr<driftless::robot::Robot>& m_robot;

  /// @brief The path to follow, as a vector of control points
  std::vector<driftless::control::Point>& m_path;

  /// @brief The target velocity to follow the path at, in inches per second
  double m_velocity;
};
}  // namespace path
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
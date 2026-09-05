#ifndef __GO_TO_POINT_COMMAND_HPP__
#define __GO_TO_POINT_COMMAND_HPP__

#include <memory>

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

/// @brief Namespace for motion control commands
/// @author Matthew Backman
namespace motion {

/// @brief Struct representing a command to go to a specific point
/// @author Matthew Backman
struct GoToPointCommand {
  /// @brief A reference to the robot to go to a point
  std::shared_ptr<driftless::robot::Robot>& m_robot;

  /// @brief The target velocity to go to the point at, in inches per second
  double m_velocity;

  /// @brief The desired x coordinate of the point to go to, in inches
  double m_x;

  /// @brief The desired y coordinate of the point to go to, in inches
  double m_y;
};
}  // namespace motion
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
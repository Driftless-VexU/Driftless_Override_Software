#ifndef __DRIVE_STRAIGHT_COMMAND_HPP__
#define __DRIVE_STRAIGHT_COMMAND_HPP__

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

/// @brief Container representing a control system command to drive a robot
/// straight in a given direction
/// @author Matthew Backman
struct DriveStraightCommand {
  /// @brief A reference to the robot to drive straight
  std::shared_ptr<driftless::robot::Robot>& m_robot;

  /// @brief The target velocity to drive at, in inches per second
  double m_velocity;

  /// @brief The desired distance to drive straight for, in inches
  double m_distance;

  /// @brief The desired angle to drive towards, in radians
  double m_theta;
};
}  // namespace motion
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
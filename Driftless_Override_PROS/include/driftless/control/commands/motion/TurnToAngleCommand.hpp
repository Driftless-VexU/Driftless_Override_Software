#ifndef __TURN_TO_ANGLE_COMMAND_HPP__
#define __TURN_TO_ANGLE_COMMAND_HPP__

#include <memory>

#include "driftless/control/motion/ETurnDirection.hpp"
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

/// @brief Struct representing a command to turn to a specific angle
/// @author Matthew Backman
struct TurnToAngleCommand {
  /// @brief A reference to the robot to turn to an angle
  std::shared_ptr<driftless::robot::Robot>& m_robot;

  /// @brief The target angular velocity to turn to the angle at, in inches per
  /// second
  double m_angular_velocity;

  /// @brief The desired angle to turn to, in radians
  double m_theta;

  /// @brief The direction to turn in, defaults to AUTO
  control::motion::ETurnDirection m_direction{
      control::motion::ETurnDirection::AUTO};
};
}  // namespace motion
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
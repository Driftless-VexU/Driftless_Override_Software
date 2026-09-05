#ifndef __GO_TO_POSE_COMMAND_HPP__
#define __GO_TO_POSE_COMMAND_HPP__

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

/// @brief Struct representing a command to go to a specific pose
/// @author Matthew Backman
struct GoToPoseCommand {
  /// @brief A reference to the robot to go to a pose
  std::shared_ptr<driftless::robot::Robot>& m_robot;

  /// @brief The target linear velocity to go to the pose at, in inches per
  /// second
  double m_linear_velocity;

  /// @brief The target linear acceleration to go to the pose at, in inches per
  /// second per second
  double m_linear_acceleration;

  /// @brief The target angular velocity to go to the pose at, in radians per
  /// second
  double m_angular_velocity;

  /// @brief The desired x coordinate of the pose, in inches
  double m_x;

  /// @brief The desired y coordinate of the pose, in inches
  double m_y;

  /// @brief The desired angle of the pose, in radians
  double m_theta;
};
}  // namespace motion
}  // namespace commands
}  // namespace control
}  // namespace driftless
#endif
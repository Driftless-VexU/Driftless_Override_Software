#ifndef __SET_ANGULAR_VELOCITY_COMMAND_HPP__
#define __SET_ANGULAR_VELOCITY_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief Namespace containing commands for the holonomic drivetrain subsystem
/// @author Matthew Backman
namespace holonomic_drive_train {

/// @brief Struct containing information needed to command the holonomic
/// drivetrain to set its angular velocity
/// @author Matthew Backman
struct SetAngularVelocityCommand {
  /// @brief The desired angular velocity of the robot (rad/s)
  const double m_angular_velocity;
  
  /// @brief Whether the angular velocity is normalized (between -1 and 1)
  const bool m_is_normal;
};
}  // namespace holonomic_drive_train
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
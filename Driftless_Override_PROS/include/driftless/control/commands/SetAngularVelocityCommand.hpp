#ifndef __SET_ANGULAR_VELOCITY_COMMAND_HPP__
#define __SET_ANGULAR_VELOCITY_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief The namespace for commands which can be sent to the control system
/// @author Matthew Backman
namespace commands {

/// @brief Struct representing a command to set the max angular velocity of a
/// control algorithm
/// @author Matthew Backman
struct SetAngularVelocityCommand {
  /// @brief The max angular velocity to set, in radians per second
  double m_angular_velocity;
};
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
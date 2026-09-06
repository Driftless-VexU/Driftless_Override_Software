#ifndef __SET_LINEAR_VELOCITY_COMMAND_HPP__
#define __SET_LINEAR_VELOCITY_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief The namespace for commands which can be sent to the control system
/// @author Matthew Backman
namespace commands {

/// @brief Struct representing a command to set the max linear velocity of a
/// control algorithm
/// @author Matthew Backman
struct SetLinearVelocityCommand {
  /// @brief The max linear velocity to set, in inches per second
  double m_linear_velocity;
};
}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif
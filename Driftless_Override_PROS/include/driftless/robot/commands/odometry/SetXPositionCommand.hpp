#ifndef __SET_X_POSITION_COMMAND_HPP__
#define __SET_X_POSITION_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief Namespace containing commands for the odometry subsystem
/// @author Matthew Backman
namespace odometry {

/// @brief Sets the X position of the robot in the odometry subsystem
/// @author Matthew Backman
struct SetXPositionCommand {
  /// @brief The x position to set the robot to (in)
  const double m_x_position;
};
}  // namespace odometry
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
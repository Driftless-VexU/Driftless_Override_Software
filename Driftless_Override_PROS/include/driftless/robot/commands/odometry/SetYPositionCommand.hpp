#ifndef __SET_Y_POSITION_COMMAND_HPP__
#define __SET_Y_POSITION_COMMAND_HPP__

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

/// @brief Sets the Y position of the robot in the odometry subsystem
/// @author Matthew Backman
struct SetYPositionCommand {
  /// @brief The y position to set the robot to (in)
  const double m_y_position;
};
}  // namespace odometry
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
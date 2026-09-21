#ifndef __SET_POSITION_COMMAND_HPP__
#define __SET_POSITION_COMMAND_HPP__

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

/// @brief Command to manually set the position of the odometry
/// @author Matthew Backman
struct SetPositionCommand {
  /// @brief The x position to set the robot to (in)
  const double m_x_pos;

  /// @brief The y position to set the robot to (in)
  const double m_y_pos;

  /// @brief The heading to set the robot to (rad)
  const double m_theta_pos;
};
}  // namespace odometry
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
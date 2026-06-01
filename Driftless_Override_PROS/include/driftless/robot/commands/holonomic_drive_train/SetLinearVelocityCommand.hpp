#ifndef __SET_LINEAR_VELOCITY_COMMAND_HPP__
#define __SET_LINEAR_VELOCITY_COMMAND_HPP__

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
class SetLinearVelocityCommand {
 private:
  double m_x_velocity;

  double m_y_velocity;

  bool m_is_normal;

 public:
  /// @brief Constructs a new SetLinearVelocityCommand with the given velocities
  /// @param x_velocity __double__ The x velocity of the motion vector
  /// @param y_velocity __double__ The y velocity of the motion vector
  /// @param is_normal __bool__ Whether the velocities are normalized (between
  /// -1 and 1)
  SetLinearVelocityCommand(double x_velocity, double y_velocity,
                           bool is_normal = false);

  bool isNormal() const;

  double getXVelocity() const;

  double getYVelocity() const;
};
}  // namespace holonomic_drive_train
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
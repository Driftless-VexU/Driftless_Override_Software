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

class SetAngularVelocityCommand {
 private:
  double m_angular_velocity;

  bool m_is_normal;

 public:
  /// @brief Constructs a new SetAngularVelocityCommand with the given angular
  /// velocity
  /// @param angular_velocity __double__ The desired angular velocity of the
  /// robot
  /// @param is_normal __bool__ Whether the angular velocity is normalized
  /// (between -1 and 1)
  SetAngularVelocityCommand(double angular_velocity, bool is_normal = false);

  bool isNormal() const;

  double getAngularVelocity() const;
};
}  // namespace holonomic_drive_train
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
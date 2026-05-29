#ifndef __SET_MOTION_VECTOR_COMMAND_HPP__
#define __SET_MOTION_VECTOR_COMMAND_HPP__

#include "driftless/robot/commands/ICommand.hpp"
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

/// @brief Command for setting the motion vector of the holonomic drivetrain
/// @author Matthew Backman
class SetMotionVectorCommand : public ICommand {
 private:
  double m_x_velocity;

  double m_y_velocity;

  double m_angular_velocity;

 public:
  /// @brief Constructs a new SetMotionVectorCommand with the given velocities
  /// @param x_velocity __double__ The x velocity of the motion vector
  /// @param y_velocity __double__ The y velocity of the motion vector
  /// @param angular_velocity __double__ The angular velocity of the motion
  /// vector
  SetMotionVectorCommand(double x_velocity, double y_velocity,
                         double angular_velocity);

  double getXVelocity() const;

  double getYVelocity() const;

  double getAngularVelocity() const;
};
}  // namespace holonomic_drive_train
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
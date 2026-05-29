#ifndef __SET_X_POSITION_COMMAND_HPP__
#define __SET_X_POSITION_COMMAND_HPP__

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

/// @brief Namespace containing commands for the odometry subsystem
/// @author Matthew Backman
namespace odometry {

/// @brief Sets the X position of the robot in the odometry subsystem
/// @author Matthew Backman
class SetXPositionCommand : public ICommand {
 private:
  double m_x_position;

 public:
  /// @brief Constructs a new SetXPositionCommand with the given X position
  /// @param x_position __double__ The X position to set the robot to
  explicit SetXPositionCommand(double x_position);

  subsystems::ESubsystem getTarget() const override;

  double getXPosition() const;
};
}  // namespace odometry
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
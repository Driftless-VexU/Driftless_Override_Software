#ifndef __SET_POSITION_COMMAND_HPP__
#define __SET_POSITION_COMMAND_HPP__

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

class SetPositionCommand : public ICommand {
 private:
  double m_x_pos;

  double m_y_pos;

  double m_theta_pos;

 public:
  /// @brief Construct a new Set Position Command object
  /// @param x_pos The x position to set the odometry to
  /// @param y_pos The y position to set the odometry to
  /// @param theta_pos The theta position to set the odometry to
  SetPositionCommand(double x_pos, double y_pos, double theta_pos);

  subsystems::ESubsystem getTarget() const override;

  double getXPos() const;

  double getYPos() const;

  double getThetaPos() const;
};
}  // namespace odometry
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
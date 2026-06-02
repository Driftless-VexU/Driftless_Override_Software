#ifndef __TANK_DRIVE_TRAIN_SET_VOLTAGE_COMMAND_HPP__
#define __TANK_DRIVE_TRAIN_SET_VOLTAGE_COMMAND_HPP__

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
namespace tank_drive_train {

  /// @brief Command to set the target voltage of a tank drive train
  /// @author Matthew Backman
  struct SetVoltageCommand {
    /// @brief The desired voltage of the left side of the drivetrain (V)
    const double m_left_voltage;

    /// @brief The desired voltage of the right side of the drivetrain (V)
    const double m_right_voltage;
  };
}
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
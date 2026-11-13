#ifndef __SET_INTAKE_MOTOR_VOLTAGE_COMMAND_HPP__
#define __SET_INTAKE_MOTOR_VOLTAGE_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the brakes subsystem
/// @author Jacob Fiedorowicz
namespace intake {

struct SetIntakeMotorVoltageCommand {
  /// @brief Voltage of the motor
  float m_voltage;
};
}  // namespace intake
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
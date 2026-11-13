#ifndef __I_INTAKE_HPP__
#define __I_INTAKE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the intake subsystem code
/// @author Jacob Fiedorowicz
namespace intake {

/// @brief Interface for intake systems
/// @author Jacob Fiedorowicz
class IIntake {
 public:
  /// @brief Destory the IIntake object
  virtual ~IIntake() = default;

  /// @brief Initialize the IIntake object
  virtual void init() = 0;

  /// @brief Run the IIntake object
  virtual void run() = 0;

  /// @brief Sets the state of the intake piston
  /// @param deployed __bool__ Whether the intake piston is deployed or not.
  /// True for deployed, false for retracted.
  virtual void setIntakePistonUpDown(bool deployed) = 0;

  /// @brief Toggle the state of the intake piston
  virtual void toggleIntakePistonUpDown() = 0;

  /// @brief Determine if the intake piston is deployed
  /// @return __bool__ Whether the intake piston is deployed or not
  virtual bool isIntakePistonDeployed() = 0;

  /// @brief Sets the voltage going to the motor
  /// @param voltage __float__ The volts sent to the motor
  virtual void setIntakeMotorVoltage(float voltage) = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
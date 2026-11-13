#ifndef __INTAKE_HPP__
#define __INTAKE_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the intake subsystem
/// @author Jacob Fiedorowicz
namespace intake {

// builder declaration, see IntakeBuilder.hpp for definition
class IntakeBuilder;

/// @brief Intake implementation
/// @author Jacob Fiedorowicz
class Intake : public IIntake {
  friend class IntakeBuilder;

 public:
  /// @brief Initialize the Intake object
  virtual void init() override;

  /// @brief Run the Intake object
  virtual void run() override;

  /// @brief Sets the state of the intake piston
  /// @param deployed __bool__ Whether the intake piston is deployed or not
  virtual void setIntakePistonUpDown(bool deployed) override;

  /// @brief Toggles the state of the intake piston
  virtual void toggleIntakePistonUpDown() override;

  /// @brief Determine if the intake piston is deployed
  /// @return __bool__ Whether the intake piston is deployed or not
  virtual bool isIntakePistonDeployed() override;

  /// @brief Sets the voltage going to the motor
  /// @param voltage __float__ The volts sent to the motor
  virtual void setIntakeMotorVoltage(float voltage) override;

 private:
  std::unique_ptr<io::IPiston> m_intake_piston{};
  hal::MotorGroup m_intake_motors{};

  /// @brief Construct a new Intake object
  /// @param builder __IntakeBuilder&&__ The builder creating this Intake
  /// object. Consumed on use.
  Intake(IntakeBuilder&& builder);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
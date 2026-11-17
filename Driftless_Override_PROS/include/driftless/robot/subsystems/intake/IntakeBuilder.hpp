#ifndef __INTAKE_BUILDER_HPP__
#define __INTAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/intake/Intake.hpp"

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

/// @brief Builder for Intake objects
/// @author Jacob Fiedorowicz
class IntakeBuilder {
  friend class Intake;

 public:
  /// @brief Add a piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>__ The piston to use
  /// @return __IntakeBuilder&&__ Reference to the active builder
  IntakeBuilder&& withPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Adds a motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor to use
  /// @return __IntakeBuilder&&__ Reference to the current builder
  IntakeBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Builds a new intake object
  /// @return __Intake__ The newly constructed intake object
  [[nodiscard]]
  Intake build() &&;

  /// @brief Builds a new intake object wrapped in a unique pointer
  /// @return __std::unique_ptr<Intake)__ Unique pointer to the newly
  /// constructed intake object
  [[nodiscard]]
  std::unique_ptr<Intake> buildUnique() &&;

 private:
  std::unique_ptr<io::IPiston> m_intake_piston{};

  hal::MotorGroup m_intake_motors{};
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
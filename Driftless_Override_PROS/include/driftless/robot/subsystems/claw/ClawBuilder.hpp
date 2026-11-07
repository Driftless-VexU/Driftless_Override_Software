
#ifndef __CLAW_BUILDER_HPP__
#define __CLAW_BUILDER_HPP__

#include "driftless/robot/subsystems/claw/Claw.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the brakes subsystem
/// @author Jack LaFreniere
namespace claw {

/// @brief Builder for Claw objects
/// @author Jack LaFreniere
class ClawBuilder {
  friend class Claw;

 public:
  /// @brief Add a piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>__ The piston to use
  /// @return __ClawBuilder&&__ Reference to the active builder
  ClawBuilder&& withPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Adds a motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor being added
  /// @return __ClawBuilder&&__ Pointer to the current builder
  ClawBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Adds a motor rotations to pivot radians to the builder
  /// @param motorRotationsToPivotRadians __float__ The motor rotations to pivot
  /// radians being added
  /// @return __ClawBuilder&&__ Pointer to the current builder
  ClawBuilder&& withMotorRotationsToPivotRadians(
      float motorRotationsToPivotRadians) &&;

  /// @brief Builds a new Claw object
  /// @return __Claw__ The newly constructed Claw
  /// object
  [[nodiscard]]
  Claw build() &&;

  /// @brief Builds a new Claw object wrapped in a unique pointer
  /// @return __std::unique_ptr<Claw>__ Unique pointer to the
  /// newly constructed Claw object
  [[nodiscard]]
  std::unique_ptr<Claw> buildUnique() &&;

 private:
  hal::MotorGroup m_pivot_motor{};
  std::unique_ptr<io::IPiston> m_piston{};

  float m_motor_rotations_to_pivot_radians{};
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
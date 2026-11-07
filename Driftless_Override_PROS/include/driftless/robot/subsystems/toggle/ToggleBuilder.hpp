#ifndef __TOGGLE_BUILDER_HPP__
#define __TOGGLE_BUILDER_HPP__

#include "driftless/robot/subsystems/toggle/Toggle.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the toggle subsystem code
/// @author Jacob Fiedorowicz
namespace toggle {

/// @brief Builder for toggle object
/// @author Jacob Fiedorowicz
class ToggleBuilder {
  friend class Toggle;

 public:
  /// @brief Add a motor to the builder
  /// @param piston __std::unique_ptr<io::IMotor>__ The motor to use
  /// @return __ToggleBuilder&&__ Reference to the active builder
  ToggleBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Builds a new Toggle object
  /// @return __Toggle__ The newly constructed toggle object
  [[nodiscard]]
  Toggle build() &&;

  /// @brief Builds a new Toggle object wrapped in a unique pointer
  /// @return __std::unique_ptr<Toggle>__ Unique pointer to the newly
  /// constructed toggle object
  [[nodiscard]]
  std::unique_ptr<Toggle> buildUnique() &&;

 private:
  hal::MotorGroup m_motor{};

  float m_velocity_to_voltage{1.0};
};
}  // namespace toggles
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif

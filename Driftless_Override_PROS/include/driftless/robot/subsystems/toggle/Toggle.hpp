#ifndef __TOGGLE_HPP__
#define __TOGGLE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/toggle/IToggle.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the toggles subsystem code
/// @author Jacob Fiedorowicz
namespace toggle {

class ToggleBuilder;

/// @brief Class representing a toggle subsystem
/// @author Jacob Fiedorowicz
class Toggle : public IToggle {
  friend class ToggleBuilder;

 private:
  hal::MotorGroup m_motor{};

  float m_velocity_to_voltage{1.0};

  /// @brief Constructs a toggle object from a builder
  /// @param builder __ToggleBuilder&&__ The builder being used to construct the
  /// drive
  Toggle(ToggleBuilder&& builder);

 public:
  /// @brief Initialize the Toggle object
  void init() override;

  /// @brief Run the Toggle object
  void run() override;

  /// @brief Sets the speed of the toggle motor
  void setMotorSpeed(float speed) override;
};
}  // namespace toggles
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
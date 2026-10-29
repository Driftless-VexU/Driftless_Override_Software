#ifndef __I_TOGGLE_HPP__
#define __I_TOGGLE_HPP__

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

class IToggle {
 public:
  /// @brief Destroy the IToggle object
  virtual ~IToggle() = default;

  /// @brief Initialize the IToggle object
  virtual void init() = 0;

  /// @brief Run the IToggle object
  virtual void run() = 0;

  /// @brief Sets the motor speed
  virtual void setMotorSpeed(float speed) = 0;
};
}  // namespace toggles
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
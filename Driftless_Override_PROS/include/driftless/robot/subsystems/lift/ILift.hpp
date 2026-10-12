#ifndef __I_LIFT_HPP__
#define __I_LIFT_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the lift subsystem
/// @author Jacob Fiedorowicz
namespace lift {

class ILift {
 public:
  /// @brief Destroy the ILift object
  virtual ~ILift() = default;

  /// @brief Initialize the ILift object
  virtual void init() = 0;

  /// @brief Run the ILift object
  virtual void run() = 0;

  /// @brief Set height of the lift
  /// @param height __float__ The desired height of the lift
  virtual void setHeight(float height) = 0;

  /// @brief Get the current height of the lift
  /// @return __float__ Returns the current height of the lift
  virtual float getHeight() = 0;
};
}  // namespace lift
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
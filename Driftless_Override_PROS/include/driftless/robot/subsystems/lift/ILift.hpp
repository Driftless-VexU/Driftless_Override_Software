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

/// @brief The namespace for the lift subsystem code
/// @author Matthew Backman
namespace lift {

/// @brief Interface for lift systems
/// @author Matthew Backman
class ILift {
public:
  /// @brief Destroy the ILift object
  /// @author Matthew Backman
  virtual ~ILift() = default;

  /// @brief Initialize the lift
  /// @author Matthew Backman
  virtual void init() = 0;

  /// @brief Run the lift
  /// @author Matthew Backman
  virtual void run() = 0;

  /*-----------------------------
  * Add additional behavior below
  * -----------------------------
  */
  
  /// @brief Set the Position object 
  /// @param position __float__ the position to target, in inches
  virtual void setPosition(float position) = 0;

  /// @brief Get the Position object 
  /// @return __float__ the current position of the lift, in inches
  virtual float getPosition() = 0;
};
} // namespace lift
} // namespace subsystems
} // namespace robot
} // namespace driftless

#endif
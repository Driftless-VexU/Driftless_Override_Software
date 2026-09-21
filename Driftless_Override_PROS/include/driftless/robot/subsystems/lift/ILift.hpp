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
};
} // namespace lift
} // namespace subsystems
} // namespace robot
} // namespace driftless

#endif
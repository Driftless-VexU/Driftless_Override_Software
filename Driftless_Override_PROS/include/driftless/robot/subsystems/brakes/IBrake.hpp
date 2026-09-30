#ifndef __I_BRAKE_HPP__
#define __I_BRAKE_HPP__

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
/// @author Matthew Backman
namespace brakes {

class IBrake {
 public:
  /// @brief Destroy the IBrake object
  virtual ~IBrake() = default;

  /// @brief Initialize the IBrake object
  virtual void init() = 0;

  /// @brief Run the IBrake object
  virtual void run() = 0;

  /// @brief Set the state of the brakes
  /// @param deployed __bool__ Whether the brakes are deployed or not. True for
  /// deployed, false for retracted.
  virtual void setState(bool deployed) = 0;

  /// @brief Toggle the state of the brakes. I.e. retract them if deployed,
  /// deploy them if retracted.
  virtual void toggleState() = 0;

  /// @brief Determine if the brakes are deployed
  /// @return __bool__ Whether the brakes are deployed or not. True for
  /// deployed, false for retracted.
  virtual bool isDeployed() = 0;
};
}  // namespace brakes
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
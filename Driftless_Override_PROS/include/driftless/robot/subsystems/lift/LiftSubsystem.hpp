#ifndef __LIFT_SUBSYSTEM_HPP__
#define __LIFT_SUBSYSTEM_HPP__

#include "driftless/robot/subsystems/ASubsystem.hpp"

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

/// @brief Subsystem to control the lift driver
class LiftSubsystem : public ASubsystem {
 public:
  // ADD CONSTRUCTOR

  /// @brief Initializes the lift subsystem
  void init() override;

  /// @brief Runs the lift subsystem
  void run() override;

  /// @brief Sends a command to the lift
  /// @param command __commands::Command&__ The command to send to the lift
  void command(const commands::Command& command) override;

  /// @brief Retrieves a state from the lift
  /// @return __void*__ The requested state of the lift
  void* state(const ESubsystemState) override;

 private:
};
}  // namespace lift
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
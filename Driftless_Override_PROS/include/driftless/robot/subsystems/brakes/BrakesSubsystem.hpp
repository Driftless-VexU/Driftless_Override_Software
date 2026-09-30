#ifndef __BRAKES_SUBSYSTEM_HPP__
#define __BRAKES_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/brakes/IBrake.hpp"

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

/// @brief Class to adapt a brake driver to the subsystem structure
/// @author Matthew Backman
class BrakesSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Brakes Subsystem object
  /// @param brake_driver __std::unique_ptr<IBrake>__ The brakes driver class to
  /// use by the subsystem
  BrakesSubsystem(std::unique_ptr<IBrake> brake_driver);

  /// @brief Initializes the brakes subsystem 
  void init() override;

  /// @brief Runs the brakes subsystem
  void run() override;

  /// @brief Send a command to the brakes subsystem  
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieves a state from the brakes subsystem  
  /// @param state __ESubsystemState__ The state to retrieve from the brake subsystem
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IBrake> m_brake_driver{};

  /// @brief Handles the set brake state command
  /// @param cmd __const commands::brakes::SetBrakeStateCommand__ The command to
  /// handle
  void handleCommand(const commands::brakes::SetBrakeStateCommand cmd);

  /// @brief Handles the toggle brake state command
  /// @param cmd __comst commands::brakes::ToggleBrakeStateCommand__ The command
  /// to handle
  void handleCommand(const commands::brakes::ToggleBrakeStateCommand cmd);

  /// @brief Handles any commands that are not explicitly handled by the brakes
  /// subsystem
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};
}  // namespace brakes
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
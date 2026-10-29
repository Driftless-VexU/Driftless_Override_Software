#ifndef __TOGGLE_SUBSYSTEM_HPP__
#define __TOGGLE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
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

/// @brief The namespace for the brakes subsystem
/// @author Jacob Fiedorowicz
namespace toggle {

/// @brief Class to adapt a brake driver to the subsystem structure
/// @author Jacob Fiedorowicz
class ToggleSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new toggle subsystem
  /// @param toggle_driver __std::unique_ptr<IToggle> the toggle driver class to
  /// use by the subsystem
  ToggleSubsystem(std::unique_ptr<IToggle> toggle_driver);

  /// @brief Initializes the toggle subsystem
  void init() override;

  /// @brief Runs the toggle subsystem
  void run() override;

  /// @brief Send a command to the toggle subsystem
  void command(const commands::Command& cmd) override;

  /// @brief Makes a state method
  void* state(ESubsystemState state) override;
 private:
  std::unique_ptr<IToggle> m_toggle_driver{};

  /// @brief Handles the set motor speed command
  /// @param cmd __const commands::toggle::SetToggleSpeedCommand__ The command
  /// to handle
  void handleCommand(const commands::toggle::SetToggleSpeedCommand cmd);

  /// @brief Handles any commands that are not explicitly handled by the brakes
  /// subsystem
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};
}  // namespace toggles
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
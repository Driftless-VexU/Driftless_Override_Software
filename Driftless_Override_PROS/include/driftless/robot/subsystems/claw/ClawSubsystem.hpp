#ifndef __CLAW_SUBSYSTEM_HPP__
#define __CLAW_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/claw/IClaw.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the claw subsystem
/// @author Jack LaFreniere
namespace claw {

/// @brief Class to adapt a claw driver to the subsystem structure
/// @author Jack LaFreniere
/// @author Matthew Backman
class ClawSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Claw Subsystem object
  /// @param claw_driver __std::unique_ptr<IClaw>__ The claw driver class to
  /// use by the subsystem
  ClawSubsystem(std::unique_ptr<IClaw> claw_driver);

  /// @brief Initialize the claw subsystem
  void init() override;

  /// @brief Run the claw subsystem
  void run() override;

  /// @brief Send a command to the claw subsystem
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieve a state from the claw subsystem
  /// @param state __ESubsystemState__ The state to retrieve from the claw
  /// subsystem
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IClaw> m_claw_driver{};

  /// @brief Handles the set up/down command of the claw
  /// @param cmd __const commands::claw::SetClawUpDownCommand The command
  /// to handle
  void handleCommand(const commands::claw::SetClawUpDownCommand cmd);

  /// @brief Handles the set open/close command of the claw
  /// @param cmd __const commands::claw::SetClawOpenCloseCommand The command to
  /// handle
  void handleCommand(const commands::claw::SetClawOpenCloseCommand cmd);

  /// @brief Handles the toggle open/close command of the claw
  /// @param cmd __const commands::claw::ToggleClawOpenCloseCommand The command
  /// to handle
  void handleCommand(const commands::claw::ToggleClawOpenCloseCommand cmd);

  /// @brief Handle the toggle up/down command of the claw
  /// @param cmd __const commands::claw::ToggleClawUpDownCommand__ The command
  /// to handle
  void handleCommand(const commands::claw::ToggleClawUpDownCommand cmd);

  /// @brief Handles any commands that are not explicitly handled by the claw
  /// subsystem
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
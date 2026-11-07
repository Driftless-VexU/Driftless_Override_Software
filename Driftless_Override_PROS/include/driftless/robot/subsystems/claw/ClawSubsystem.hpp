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
class ClawSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Claw Subsystem object
  /// @param claw_driver __std::unique_ptr<IClaw>__ The claw driver class to
  /// use by the subsystem
  ClawSubsystem(std::unique_ptr<IClaw> claw_driver);

  /// @brief Initializes the claw subsystem 
  void init() override;

  /// @brief Runs the claw subsystem
  void run() override;

  /// @brief Send a command to the claw subsystem  
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieves a state from the claw subsystem  
  /// @param state __ESubsystemState__ The state to retrieve from the claw subsystem
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IClaw> m_claw_driver{};
  
  /// @brief Handles the up/down state of the claw
  /// @param cmd __comst commands::claw::SetClawUpDownCommand The command
  /// to handle
  void handleCommand(const commands::claw::SetClawUpDownCommand cmd);
  
  /// @brief Handles the open/close state of the claw
  /// @param cmd __const commands::claw::SetClawOpenCloseCommand The command to
  /// handle
  void handleCommand(const commands::claw::SetClawOpenCloseCommand cmd);

  /// @brief Handles the open/close state of the claw
  /// @param cmd __comst commands::claw::ToggleClawOpenCloseCommand The command
  /// to handle
  void handleCommand(const commands::claw::ToggleClawOpenCloseCommand cmd);

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
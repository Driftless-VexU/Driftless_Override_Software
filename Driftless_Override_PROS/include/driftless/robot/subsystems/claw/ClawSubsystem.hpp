#ifndef __CLAW_SUBSYSTEM_HPP__
#define __CLAW_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
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

/// @brief Namespace containing the claw subsystem
/// @author Matthew Backman
namespace claw {

/// @brief Adapts a claw driver to the subsystem ecosystem
/// @author Matthew Backman
class ClawSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Claw Subsystem object
  /// @param claw __std::unique_ptr<IClaw>__ the claw driver to use
  ClawSubsystem(std::unique_ptr<IClaw> claw);

  /// @brief Initialize the claw subsystem
  void init() override;

  /// @brief Run the claw subsystem
  void run() override;

  /// @brief Send a command to the claw subsystem
  /// @param cmd __const commands::Command&__ The command to send to the claw
  void command(const commands::Command& cmd) override;

  /// @brief Retrieve a state from the claw subsystem
  /// @param state_name __const ESubsystemState__ The state to retrieve
  /// @return __void*__ The value of the desired state
  void* state(const ESubsystemState state_name) override;

 private:
  std::unique_ptr<IClaw> m_claw_driver{};

  /// @brief Handles the set elbow rotation command
  /// @param cmd __const commands::claw::ClawSetElbowRotationCommand__ The
  /// command to handle
  void handleCommand(const commands::claw::ClawSetElbowRotationCommand cmd);

  /// @brief Handles the set wrist rotation command
  /// @param cmd __const commands::claw::ClawSetWristRotationCommand__ The
  /// command to handle
  void handleCommand(const commands::claw::ClawSetWristRotationCommand cmd);

  /// @brief Handles the flip command
  /// @param cmd __const commands::claw::ClawFlipCommand__ The command to handle
  void handleCommand(const commands::claw::ClawFlipCommand cmd);

  /// @brief Handles any commands not explicitly handled
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};

}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
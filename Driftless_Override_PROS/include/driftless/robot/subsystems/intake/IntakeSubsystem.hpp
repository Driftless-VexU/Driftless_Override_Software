#ifndef __INTAKE_SUBSYSTEM_HPP__
#define __INTAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the intake subsystem
/// @author Jacob Fiedorowicz
namespace intake {

/// @brief Class to adapt an intake driver to the subsystem structure
/// @author Jacob Fiedorowicz
class IntakeSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Intake subsystem object
  /// @param intake_driver __std::unique_ptr<IIntake>__ The intake driver class
  /// to use by the subsystem
  IntakeSubsystem(std::unique_ptr<IIntake> intake_driver);

  /// @brief Initializes the intake subsystem
  void init() override;

  /// @brief Runs the intake subsystem
  void run() override;

  /// @brief Send a command to the intake subsystem
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieves a state from the intake subsystem
  /// @param state __ESubsystemState__ The state to retrieve from the intake
  /// subsystem
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IIntake> m_intake_driver{};

  /// @brief handles the set intake piston heigght command
  /// @param cmd __const commands::intake::SetIntakePistonHeightCommand__ the
  /// command to handle
  void handleCommand(const commands::intake::SetIntakePistonHeightCommand cmd);

  /// @brief handles the toggle intake piston height command
  /// @param cmd __const commands::intake::ToggleIntakePistonHeightCommand__ the
  /// command to handle
  void handleCommand(
      const commands::intake::ToggleIntakePistonHeightCommand cmd);

  /// @brief handles the set intake motor voltagecommand
  /// @param cmd __const commands::intake::SetIntakeMotorVoltageCommand__ the
  /// command to handle
  void handleCommand(const commands::intake::SetIntakeMotorVoltageCommand cmd);

  /// @brief Handles any commands that are not explicitly handled by the intake
  /// subsystem
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
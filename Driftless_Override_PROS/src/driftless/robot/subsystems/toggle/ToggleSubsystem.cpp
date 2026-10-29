#include "driftless/robot/subsystems/toggle/ToggleSubsystem.hpp"

#include "driftless/robot/commands/toggle/SetToggleSpeedCommand.hpp"
#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/toggle/IToggle.hpp"


namespace driftless::robot::subsystems::toggle {
ToggleSubsystem::ToggleSubsystem(std::unique_ptr<IToggle> toggle)
    : ASubsystem{ESubsystem::TOGGLE}, m_toggle_driver{std::move(toggle)} {}
void ToggleSubsystem::init() {}

void ToggleSubsystem::run() {}

void ToggleSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void ToggleSubsystem::handleCommand(
    const commands::toggle::SetToggleSpeedCommand cmd) {
  m_toggle_driver->setMotorSpeed(cmd.m_motor_speed);
}
void ToggleSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the toggle subsystem");
}
}  // namespace driftless::robot::subsystems::toggle
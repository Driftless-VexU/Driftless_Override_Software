#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace driftless::robot::subsystems::intake {
IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IIntake> intake)
    : ASubsystem{ESubsystem::INTAKE}, m_intake_driver{std::move(intake)} {}

void IntakeSubsystem::init() {}

void IntakeSubsystem::run() {}

void IntakeSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}
void* IntakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  if (state_name == ESubsystemState::INTAKE_IS_DEPLOYED) {
    result = new bool{m_intake_driver->isPistonDeployed()};
  }
  return result;
}
void IntakeSubsystem::handleCommand(
    const commands::intake::SetIntakePistonStateCommand cmd) {
  m_intake_driver->setPistonState(cmd.m_deployed);
}
void IntakeSubsystem::handleCommand(
    const commands::intake::ToggleIntakePistonStateCommand cmd) {
  m_intake_driver->togglePistonState();
}
void IntakeSubsystem::handleCommand(
    const commands::intake::SetIntakeMotorSpeedCommand cmd) {
  m_intake_driver->setMotorSpeed(cmd.m_speed);
}
void IntakeSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the intake subsystem");
}
}  // namespace driftless::robot::subsystems::intake
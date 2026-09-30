#include "driftless/robot/subsystems/brakes/BrakesSubsystem.hpp"

namespace driftless::robot::subsystems::brakes {
BrakesSubsystem::BrakesSubsystem(std::unique_ptr<IBrake> brake)
    : ASubsystem{ESubsystem::BRAKES}, m_brake_driver{std::move(brake)} {}

void BrakesSubsystem::init() {}

void BrakesSubsystem::run() {}

void BrakesSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* BrakesSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  if (state_name == ESubsystemState::BRAKES_IS_DEPLOYED) {
    result = new bool{m_brake_driver->isDeployed()};
  }
  return result;
}
void BrakesSubsystem::handleCommand(const commands::brakes::SetBrakeStateCommand cmd) {
  m_brake_driver->setState(cmd.m_deployed);
}
void BrakesSubsystem::handleCommand(const commands::brakes::ToggleBrakeStateCommand cmd) {
  m_brake_driver->toggleState();
}
void BrakesSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the brakes subsystem");
}
}   // namespace driftless::robot::subsystems::brakes
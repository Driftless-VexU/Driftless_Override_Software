#include "driftless/robot/subsystems/lift/LiftSubsystem.hpp"

namespace driftless::robot::subsystems::lift {
LiftSubsystem::LiftSubsystem(std::unique_ptr<ILift> lift)
    : ASubsystem{ESubsystem::LIFT}, m_lift_driver{std::move(lift)} {}

void LiftSubsystem::init() {}

void LiftSubsystem::run() {}

void LiftSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* LiftSubsystem::state(ESubsystemState state_name) {
  void* result{};
  if (state_name == ESubsystemState::LIFT_GET_HEIGHT) {
    if (m_lift_driver) {
    }
  }
}
void LiftSubsystem::handleCommand(
    const commands::lift::SetLiftHeightCommand cmd) {
  m_lift_driver->setHeight(cmd.m_height);
}
void LiftSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the lift subsystem");
}
}  // namespace driftless::robot::subsystems::lift
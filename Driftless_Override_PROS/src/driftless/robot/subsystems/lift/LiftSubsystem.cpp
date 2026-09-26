#include "driftless/robot/subsystems/lift/LiftSubsystem.hpp"

#include "driftless/robot/subsystems/ESubsystemState.hpp"


namespace driftless::robot::subsystems::lift {

LiftSubsystem::LiftSubsystem(std::unique_ptr<ILift>& lift)
    : ASubsystem(ESubsystem::LIFT), m_lift{std::move(lift)} {}

void LiftSubsystem::init() {}

void LiftSubsystem::run() {}

void LiftSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* LiftSubsystem::state(const ESubsystemState state_name) {
  void* output{};

  if (state_name == ESubsystemState::LIFT_GET_POSITION) {
    output = new float(m_lift->getPosition());
  }

  return output;
}

void LiftSubsystem::handleCommand(
    const commands::lift::SetPositionCommand cmd) {
  m_lift->setPosition(cmd.m_position);
}
}  // namespace driftless::robot::subsystems::lift
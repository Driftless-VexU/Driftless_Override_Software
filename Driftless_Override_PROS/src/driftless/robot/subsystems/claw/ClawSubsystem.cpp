#include "driftless/robot/subsystems/claw/ClawSubsystem.hpp"

namespace driftless::robot::subsystems::claw {
ClawSubsystem::ClawSubsystem(std::unique_ptr<IClaw> claw)
    : m_claw_driver{std::move(claw)} {}

void ClawSubsystem::init() { m_claw_driver->init(); }

void ClawSubsystem::run() { m_claw_driver->run(); }

void ClawSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { handleCommand(cmd); }, cmd);
}

void* ClawSubsystem::state(const ESubsystemState state_name) {
  void* out;
  if (state_name == ESubsystemState::CLAW_GET_ELBOW_ROTATION) {
    out = new float(m_claw_driver->getElbowRotation());
  } else if (state_name == ESubsystemState::CLAW_GET_WRIST_ROTATION) {
    out = new float(m_claw_driver->getWristRotation());
  } else if (state_name == ESubsystemState::CLAW_IS_FLIPPED) {
    out = new bool(m_claw_driver->isClawFlipped());
  }

  return out;
}

void ClawSubsystem::handleCommand(
    const commands::claw::ClawSetElbowRotationCommand cmd) {
  m_claw_driver->setElbowRotation(cmd.m_rotation);
}

void ClawSubsystem::handleCommand(
    const commands::claw::ClawSetWristRotationCommand cmd) {
  m_claw_driver->setWristRotation(cmd.m_rotation);
}

void ClawSubsystem::handleCommand(const commands::claw::ClawFlipCommand cmd) {
  m_claw_driver->flipClaw();
}

void ClawSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the claw subsystem");
}
}  // namespace driftless::robot::subsystems::claw
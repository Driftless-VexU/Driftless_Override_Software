#include "driftless/robot/subsystems/claw/ClawSubsystem.hpp"

namespace driftless::robot::subsystems::claw {
ClawSubsystem::ClawSubsystem(std::unique_ptr<IClaw> claw_driver)
    : m_claw_driver{std::move(claw_driver)} {}

void ClawSubsystem::init() { m_claw_driver->init(); }

void ClawSubsystem::run() { m_claw_driver->run(); }

void ClawSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* ClawSubsystem::state(ESubsystemState state) {
  void* out{nullptr};

  if (state == ESubsystemState::CLAW_IS_UPRIGHT) {
    out = new bool(m_claw_driver->isClawUp());
  }

  return out;
}

void ClawSubsystem::handleCommand(const commands::claw::SetClawUpDownCommand cmd) {
  m_claw_driver->setUpDown(cmd.m_is_up);
}

void ClawSubsystem::handleCommand(const commands::claw::SetClawOpenCloseCommand cmd) {
  m_claw_driver->setOpenClosed(cmd.m_is_open);
}

void ClawSubsystem::handleCommand(const commands::claw::ToggleClawOpenCloseCommand cmd) {
  m_claw_driver->toggleOpenClosed();
}
}  // namespace driftless::robot::subsystems::claw
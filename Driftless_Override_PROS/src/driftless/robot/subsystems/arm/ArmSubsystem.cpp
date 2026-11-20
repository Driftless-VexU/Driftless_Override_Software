#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"

namespace driftless::robot::subsystems::arm {
ArmSubsystem::ArmSubsystem(std::unique_ptr<IArm> arm_driver)
    : ASubsystem{ESubsystem::ARM}, m_arm{std::move(arm_driver)} {}

void ArmSubsystem::init() { m_arm->init(); }

void ArmSubsystem::run() { m_arm->run(); }

void ArmSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* ArmSubsystem::state(ESubsystemState state) {
  void* output{nullptr};

  if (state == ESubsystemState::ARM_GET_CURRENT_STATE) {
    output =
        new int{static_cast<int>(m_arm->getCurrentPositionState())};
  }

  return output;
}

void ArmSubsystem::handleCommand(const commands::arm::ArmGoLoadCommand cmd) {
  m_arm->goToLoadPosition();
  m_arm->enable();
}

void ArmSubsystem::handleCommand(const commands::arm::ArmGoScanCommand cmd) {
  m_arm->goToScanPosition();
  m_arm->enable();
}

void ArmSubsystem::handleCommand(
    const commands::arm::ArmGoScoreHighCommand cmd) {
  m_arm->goToScoreHighPosition();
  m_arm->enable();
}

void ArmSubsystem::handleCommand(
    const commands::arm::ArmGoScoreLowCommand cmd) {
  m_arm->goToScoreLowPosition();
  m_arm->enable();
}

void ArmSubsystem::handleCommand(const commands::arm::ArmResetCommand cmd) {
  m_arm->reset();
  m_arm->enable();
}

void ArmSubsystem::handleCommand(const commands::misc::DisableCommand cmd) {
  m_arm->disable();
}

void ArmSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the arm subsystem");
}

}  // namespace driftless::robot::subsystems::arm
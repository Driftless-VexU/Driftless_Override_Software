#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"

namespace driftless::robot::subsystems::arm {
ArmSubsystem::ArmSubsystem(std::unique_ptr<IArm> arm_driver)
    : ASubsystem{ESubsystem::ARM}, m_arm{std::move(arm_driver)} {}

void ArmSubsystem::init() { m_arm->init(); }

void ArmSubsystem::run() { m_arm->run(); }

void ArmSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* ArmSubsystem::state(ESubsystemState state) { return nullptr; }

void ArmSubsystem::handleCommand(const commands::arm::ArmGoLoadCommand cmd) {
  m_arm->goToLoadPosition();
}

void ArmSubsystem::handleCommand(const commands::arm::ArmGoScanCommand cmd) {
  m_arm->goToScanPosition();
}

void ArmSubsystem::handleCommand(
    const commands::arm::ArmGoScoreHighCommand cmd) {
  m_arm->goToScoreHighPosition();
}

void ArmSubsystem::handleCommand(
    const commands::arm::ArmGoScoreLowCommand cmd) {
  m_arm->goToScoreLowPosition();
}

void ArmSubsystem::handleCommand(const commands::arm::ArmResetCommand cmd) {
  m_arm->reset();
}

void ArmSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the arm subsystem");
}
}  // namespace driftless::robot::subsystems::arm
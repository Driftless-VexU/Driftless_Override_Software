#include "driftless/robot/subsystems/aligner/AlignerSubsystem.hpp"

#include "driftless/robot/commands/aligner/SetAlignerStateCommand.hpp"
#include "driftless/robot/commands/aligner/ToggleAlignerStateCommand.hpp"

namespace driftless::robot::subsystems::aligner {
AlignerSubsystem::AlignerSubsystem(std::unique_ptr<IAligner> aligner)
    : ASubsystem{ESubsystem::ALIGNER}, m_aligner_driver{std::move(aligner)} {}

void AlignerSubsystem::init() { m_aligner_driver->init(); }

void AlignerSubsystem::run() { m_aligner_driver->run(); }

void AlignerSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void* AlignerSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  if (state_name == ESubsystemState::ALIGNER_IS_DEPLOYED) {
    result = new bool{m_aligner_driver->isDeployed()};
  }
  return result;
}
void AlignerSubsystem::handleCommand(
    const commands::aligner::SetAlignerStateCommand cmd) {
  m_aligner_driver->setState(cmd.m_deployed);
}
void AlignerSubsystem::handleCommand(
    const commands::aligner::ToggleAlignerStateCommand cmd) {
  m_aligner_driver->toggleState();
}
void AlignerSubsystem::handleCommand(const auto cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type " +
      std::string(typeid(cmd).name()) + " for the aligner subsystem");
}
}  // namespace driftless::robot::subsystems::aligner
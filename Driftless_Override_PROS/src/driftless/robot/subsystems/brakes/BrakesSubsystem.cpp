#include "driftless/robot/subsystems/brakes/BrakesSubsystem.hpp"

namespace driftless::robot::subsystems::brakes {
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
}  // namespace driftless::robot::subsystems::brakes
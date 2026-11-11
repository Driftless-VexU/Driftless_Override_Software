#include "driftless/robot/subsystems/wall/WallSubsystem.hpp"

namespace driftless::robot::subsystems::wall {
WallSubsystem::WallSubsystem(std::unique_ptr<IWall> wall)
    : ASubsystem{ESubsystem::WALL}, m_wall_driver{std::move(wall)} {}

void WallSubsystem::init() { m_wall_driver->init(); }

void WallSubsystem::run() { m_wall_driver->run(); }

void WallSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) { this->handleCommand(cmd); }, cmd);
}

void WallSubsystem::handleCommand(const commands::wall::SetWallStateCommand cmd) {
    m_wall_driver->setWallExtensionState(cmd.m_deployed);
}
void WallSubsystem::handleCommand(const commands::wall::ToggleWallStateCommand cmd) {
    m_wall_driver->toggleWallExtensionState();
}
void WallSubsystem::handleCommand(const commands::wall::SetWallWheelStateCommand cmd) {
    m_wall_driver->setWallWheelState(cmd.m_deployed);
}
void WallSubsystem::handleCommand(const commands::wall::ToggleWallWheelStateCommand cmd) {
    m_wall_driver->toggleWallWheelState();
}
void WallSubsystem::handleCommand(const commands::wall::SetWallWheelMotorVoltageCommand cmd) {
    m_wall_driver->setWallWheelMotorVoltage(cmd.m_voltage);
}
void WallSubsystem::handleCommand(const auto cmd) {
    throw std::invalid_argument("No behavior defined to handle the command of type " + std::string(typeid(cmd).name()) + " for the wall subsystem");
}
}  // namespace driftless::robot::subsystems::wall
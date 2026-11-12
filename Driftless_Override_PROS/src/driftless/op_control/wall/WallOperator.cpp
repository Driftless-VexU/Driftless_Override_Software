#include "driftless/op_control/wall/WallOperator.hpp"

namespace driftless::op_control::wall {
WallOperator::WallOperator(const std::shared_ptr<io::IController>& controller,
                           const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void WallOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital wall_toggle{
      profile->getDigitalControlMapping(op_control::EControl::WALL_TOGGLE)};

  EControllerDigital wall_wheel_toggle{profile->getDigitalControlMapping(
      op_control::EControl::WALL_WHEEL_TOGGLE)};

  if (m_controller->getNewDigital(wall_toggle)) {
    toggleWall();
  }
  
  if (m_controller->getDigital(wall_wheel_toggle)) {
    setWallWheelState(true);
    setWallMotorVoltage(12);
  } else {
    setWallWheelState(false);
    setWallMotorVoltage(0);
  }
}

void WallOperator::toggleWall() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::WALL, robot::commands::wall::ToggleWallStateCommand{});
}

void WallOperator::setWallWheelState(bool deployed) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::WALL, robot::commands::wall::SetWallWheelStateCommand{deployed});
}

void WallOperator::setWallMotorVoltage(float voltage) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::WALL, robot::commands::wall::SetWallWheelMotorVoltageCommand{voltage});
}
}  // namespace driftless::op_control::wall
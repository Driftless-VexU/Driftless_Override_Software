#include "driftless/robot/subsystems/wall/WallBuilder.hpp"
#include "driftless/robot/subsystems/wall/WallSubsystem.hpp"
namespace driftless::robot::subsystems::wall {
void Wall::init() {}

void Wall::run() {}

void Wall::setWallExtensionState(bool deployed) {
  if (deployed) {
    m_piston_wall->extend();
  } else {
    m_piston_wall->retract();
  }
}
void Wall::toggleWallExtensionState() { m_piston_wall->toggleState(); }

bool Wall::isWallExtended() { return m_piston_wall->isExtended(); }

void Wall::setWallWheelState(bool deployed) {
  if (deployed) {
    m_piston_wheel->extend();
  } else {
    m_piston_wheel->retract();
  }
}
void Wall::toggleWallWheelState() { m_piston_wheel->toggleState(); }

bool Wall::isWallWheelExtended() { return m_piston_wheel->isExtended(); }

void Wall::setWallWheelMotorVoltage(float voltage) {
  m_wheel_motor.setVoltage(voltage);
}

Wall::Wall(WallBuilder&& builder)
    : m_piston_wall(std::move(builder.m_piston_wall)),
      m_piston_wheel(std::move(builder.m_piston_wheel)),
      m_wheel_motor(std::move(builder.m_wheel_motor)) {}
}  // namespace driftless::robot::subsystems::wall
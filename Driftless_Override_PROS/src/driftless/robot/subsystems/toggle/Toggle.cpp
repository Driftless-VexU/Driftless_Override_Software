#include "driftless/robot/subsystems/toggle/Toggle.hpp"

#include "driftless/robot/subsystems/toggle/ToggleBuilder.hpp"
namespace driftless::robot::subsystems::toggle {
void Toggle::init() {}

void Toggle::run() {}

void Toggle::setMotorSpeed(float speed) { m_motor.setVelocity(speed); }

Toggle::Toggle(ToggleBuilder&& builder) : m_motor(std::move(builder.m_motor)) {}
}  // namespace driftless::robot::subsystems::toggle
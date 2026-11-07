#include "driftless/robot/subsystems/claw/Claw.hpp"

#include <cmath>

#include "driftless/robot/subsystems/claw/ClawBuilder.hpp"
namespace driftless::robot::subsystems::claw {
void Claw::init() {}

void Claw::run() {}

void Claw::setOpenClosed(bool isOpen) {
  if (isOpen) {
    m_piston->extend();
  } else {
    m_piston->retract();
  }
}

void Claw::setUpDown(bool isUp) {
  m_target_pivot_position = isUp * m_motor_rotations_to_pivot_radians * M_PI;
  m_isUp = isUp;
}

void Claw::toggleOpenClosed() { m_piston->toggleState(); }

bool Claw::isClawUp() { return m_isUp; }

Claw::Claw(ClawBuilder&& builder)
    : m_pivot_motor{std::move(builder.m_pivot_motor)},
      m_piston{std::move(builder.m_piston)},
      m_motor_rotations_to_pivot_radians{
          builder.m_motor_rotations_to_pivot_radians} {}
}  // namespace driftless::robot::subsystems::claw

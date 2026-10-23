#include "driftless/robot/subsystems/intake/Intake.hpp"

#include "driftless/robot/subsystems/intake/IntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
void Intake::init() { m_motor.init(); }

void Intake::run() {}

void Intake::setPistonState(bool deployed) {
  if (deployed) {
    m_piston->extend();
  } else {
    m_piston->retract();
  }
}
void Intake::togglePistonState() { m_piston->toggleState(); }

bool Intake::isPistonDeployed() { return m_piston->isExtended(); }

void Intake::setMotorSpeed(float speed) { m_motor.setVelocity(speed); }
Intake::Intake(IntakeBuilder&& builder)
    : m_piston(std::move(builder.m_piston)) {}
}  // namespace driftless::robot::subsystems::intake
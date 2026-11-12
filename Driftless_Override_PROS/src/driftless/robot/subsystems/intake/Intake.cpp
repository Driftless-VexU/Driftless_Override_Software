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

void Intake::setMotorSpeed(float speed) { m_motor.setVoltage(speed); }
Intake::Intake(IntakeBuilder&& builder)
    : m_piston(std::move(builder.m_piston)),
      m_motor(std::move(builder.m_motor)) {}
}  // namespace driftless::robot::subsystems::intake
#include "driftless/robot/subsystems/intake/Intake.hpp"

#include "driftless/robot/subsystems/intake/IntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
void Intake::init() { m_intake_motors.init(); }

void Intake::run() {}

void Intake::setIntakePistonUpDown(bool deployed) {
  if (deployed) {
    m_intake_piston->extend();
  } else {
    m_intake_piston->retract();
  }
}
void Intake::toggleIntakePistonUpDown() { m_intake_piston->toggleState(); }

bool Intake::isIntakePistonDeployed() { return m_intake_piston->isExtended(); }

void Intake::setIntakeMotorVoltage(float voltage) { m_intake_motors.setVoltage(voltage); }
Intake::Intake(IntakeBuilder&& builder)
    : m_intake_piston(std::move(builder.m_intake_piston)),
      m_intake_motors(std::move(builder.m_intake_motors)) {}
}  // namespace driftless::robot::subsystems::intake
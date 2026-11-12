#include "driftless/op_control/intake/IntakeOperator.hpp"

namespace driftless::op_control::intake {
IntakeOperator::IntakeOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void IntakeOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital intake_piston_toggle(
      profile->getDigitalControlMapping(op_control::EControl::INTAKE_TOGGLE));

  if (m_controller->getDigital(intake_piston_toggle)) {
    setIntakePistonState(true);
    setIntakeMotorSpeed(12.0f);
  }
  else
  {
    setIntakePistonState(false);
    setIntakeMotorSpeed(0.0f);
  }
}

void IntakeOperator::toggleIntakePistonState() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
    robot::commands::intake::ToggleIntakePistonStateCommand{});
}

void IntakeOperator::setIntakeMotorSpeed(float speed) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
    robot::commands::intake::SetIntakeMotorSpeedCommand{speed});
}

void IntakeOperator::setIntakePistonState(bool state) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
  robot::commands::intake::SetIntakePistonStateCommand{state});
}
}  // namespace driftless::op_control::intake
#include "driftless/op_control/intake/IntakeOperator.hpp"

namespace driftless::op_control::intake {
IntakeOperator::IntakeOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void IntakeOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital intake_run_in_btn(
      profile->getDigitalControlMapping(op_control::EControl::INTAKE_RUN_IN));

  if (m_controller->getDigital(intake_run_in_btn)) {
    setIntakePistonUpDown(true);
    setIntakeMotorVoltage(12.0f);
  }
  else
  {
    setIntakePistonUpDown(false);
    setIntakeMotorVoltage(0.0f);
  }
}

void IntakeOperator::toggleIntakePistonUpDown() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
    robot::commands::intake::ToggleIntakePistonHeightCommand{});
}

void IntakeOperator::setIntakeMotorVoltage(float voltage) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
    robot::commands::intake::SetIntakeMotorVoltageCommand{voltage});
}

void IntakeOperator::setIntakePistonUpDown(bool state) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
  robot::commands::intake::SetIntakePistonHeightCommand{state});
}
}  // namespace driftless::op_control::intake

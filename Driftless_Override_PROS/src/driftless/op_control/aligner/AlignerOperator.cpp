#include "driftless/op_control/aligner/AlignerOperator.hpp"

#include "driftless/robot/commands/aligner/ToggleAlignerStateCommand.hpp"


namespace driftless::op_control::aligner {
AlignerOperator::AlignerOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void AlignerOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital aligner_toggle(
      profile->getDigitalControlMapping(op_control::EControl::ALIGNER_TOGGLE));

  if (m_controller->getNewDigital(aligner_toggle)) {
    m_overriden = !m_overriden;
    if (m_overriden) {
      toggleAligner();
    }
  }

  if ((getLiftHeight() > 2.0f) && !m_overriden) {
    setAlignerState(true);
  } else {
    setAlignerState(false);
  }
}

void AlignerOperator::setAlignerState(bool deployed) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ALIGNER,
      robot::commands::aligner::SetAlignerStateCommand{deployed});
}

void AlignerOperator::toggleAligner() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ALIGNER,
                       robot::commands::aligner::ToggleAlignerStateCommand{});
}
}  // namespace driftless::op_control::aligner
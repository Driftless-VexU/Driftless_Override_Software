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
    toggleAligner();
  }
}

void AlignerOperator::toggleAligner() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ALIGNER,
    robot::commands::aligner::ToggleAlignerStateCommand{});
}
}  // namespace driftless::op_control::aligner
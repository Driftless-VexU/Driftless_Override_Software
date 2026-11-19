#include "driftless/op_control/aligner/AlignerOperator.hpp"
#include "driftless/robot/commands/aligner/ToggleAlignerStateCommand.hpp"

namespace driftless::op_control::aligner {
AlignerOperator::AlignerOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void AlignerOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  if (getLiftHeight() > 2.0f) {
    setAlignerState(true);
  }
  else
  {
    setAlignerState(false);
  }
}

float AlignerOperator::getLiftHeight() {
  void* state =
      m_robot->getState(robot::subsystems::ESubsystem::LIFT,
                        robot::subsystems::ESubsystemState::LIFT_GET_HEIGHT);
  if (!state) {
    throw std::runtime_error(
        "LiftOperator: Lift subsystem state LIFT_GET_HEIGHT is null");
  }
  return *static_cast<float*>(state);
}

void AlignerOperator::setAlignerState(bool deployed) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ALIGNER, robot::commands::aligner::SetAlignerStateCommand{deployed});
}

void AlignerOperator::toggleAligner() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ALIGNER,
    robot::commands::aligner::ToggleAlignerStateCommand{});
}
}  // namespace driftless::op_control::aligner
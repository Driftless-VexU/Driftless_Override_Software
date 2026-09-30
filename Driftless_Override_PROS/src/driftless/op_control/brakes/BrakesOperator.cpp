#include "driftless/op_control/brakes/BrakesOperator.hpp"

namespace driftless::op_control::brakes {
BrakesOperator::BrakesOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void BrakesOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital brakes_toggle(
      profile->getDigitalControlMapping(op_control::EControl::BRAKE_TOGGLE));

  if (m_controller->getNewDigital(brakes_toggle)) {
    toggleBrakesState();
  }
}

void BrakesOperator::toggleBrakesState() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::BRAKES,
                       robot::commands::brakes::ToggleBrakeStateCommand{});
}
}  // namespace driftless::op_control::brakes
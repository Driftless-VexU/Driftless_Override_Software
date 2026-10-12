#include "driftless/op_control/lift/LiftOperator.hpp"

namespace driftless::op_control::lift {
    LiftOperator::LiftOperator(
        const std::shared_ptr<io::IController>& controller,
        const std::shared_ptr<robot::Robot>& robot)
        : m_controller{controller}, m_robot{robot} {}

void LiftOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
        EControllerDigital lift_toggle(
            profile->getDigitalControlMapping(op_control::EControl::LIFT_TOGGLE));

    if (m_controller->getNewDigital(lift_toggle)) {
        toggleLiftHeight();
    }
}

void LiftOperator::toggleLiftHeight() {
    m_robot->sendCommand(robot::subsystems::ESubsystem::LIFT,
    robot::commands::lift::SetLiftHeightCommand{});
}
}
#include "driftless/op_control/toggle/ToggleOperator.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"

namespace driftless::op_control::toggle {
    ToggleOperator::ToggleOperator(
        const std::shared_ptr<io::IController>& controller,
        const std::shared_ptr<robot::Robot>& robot)
        : m_controller{controller}, m_robot{robot} {}

    void ToggleOperator::update(
        const std::unique_ptr<profiles::IProfile>& profile) {
            EControllerDigital toggle_speed(
                profile->getDigitalControlMapping(op_control::EControl::TOGGLE_SPEED));

            if (m_controller->getNewDigital(toggle_speed)) {
                setMotorSpeed();
            }
        }
}
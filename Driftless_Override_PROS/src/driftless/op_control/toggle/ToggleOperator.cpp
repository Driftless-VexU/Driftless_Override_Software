#include "driftless/op_control/toggle/ToggleOperator.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/commands/toggle/SetToggleSpeedCommand.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"

namespace driftless::op_control::toggle {
    ToggleOperator::ToggleOperator(
        const std::shared_ptr<io::IController>& controller,
        const std::shared_ptr<robot::Robot>& robot)
        : m_controller{controller}, m_robot{robot} {}

    void ToggleOperator::update(
        const std::unique_ptr<profiles::IProfile>& profile) {
            EControllerDigital toggle_speed(
                profile->getDigitalControlMapping(op_control::EControl::TOGGLE_SPEED));

            if (m_controller->getDigital(toggle_speed)) {
                setMotorSpeed(12.0f);
            }
            else
            {
                setMotorSpeed(0.0f);
            }
        }
        void ToggleOperator::setMotorSpeed(float speed)() {
            m_robot->sendCommand(robot::subsystems::ESubsystem::TOGGLE,
            robot::commands::toggle::SetToggleSpeedCommand{});
        }
}
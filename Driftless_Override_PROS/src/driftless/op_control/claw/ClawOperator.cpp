#include "driftless/op_control/claw/ClawOperator.hpp"

#include "driftless/robot/commands/claw/SetClawOpenCloseCommand.hpp"


namespace driftless::op_control::claw {
ClawOperator::ClawOperator(std::shared_ptr<io::IController> controller,
                           std::shared_ptr<robot::Robot> robot)
    : m_robot{robot}, m_controller{controller} {}

void ClawOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  bool do_flip{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::CLAW_FLIP))};

  bool toggle_closed{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::CLAW_TOGGLE_CLOSED))};

  bool toggle_open{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::ARM_GO_LOAD))};

  /*if (do_flip) {
    toggleClawPivot();
  }*/

  if (toggle_closed) {
    toggleClawOpenClose();
  }

  if (toggle_open) {
    setClawOpenClose(true);
  }
}

inline void ClawOperator::toggleClawPivot() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAW,
                       robot::commands::claw::ToggleClawUpDownCommand{});
}

inline void ClawOperator::toggleClawOpenClose() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAW,
                       robot::commands::claw::ToggleClawOpenCloseCommand{});
}
inline void ClawOperator::setClawOpenClose(bool openClose) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAW,
                       robot::commands::claw::SetClawOpenCloseCommand{openClose});
}
}  // namespace driftless::op_control::claw
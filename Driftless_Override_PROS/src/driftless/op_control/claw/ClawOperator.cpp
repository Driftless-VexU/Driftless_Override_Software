#include "driftless/op_control/claw/ClawOperator.hpp"

namespace driftless::op_control::claw {
ClawOperator::ClawOperator(std::shared_ptr<robot::Robot> robot,
                           std::shared_ptr<io::IController> controller)
    : m_robot{robot}, m_controller{controller} {}

void ClawOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  bool do_flip{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::CLAW_FLIP))};

  bool toggle_closed{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::CLAW_TOGGLE_CLOSED))};

  if (do_flip) {
    toggleClawPivot();
  }

  if (toggle_closed) {
    toggleClawOpenClose();
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
}  // namespace driftless::op_control::claw
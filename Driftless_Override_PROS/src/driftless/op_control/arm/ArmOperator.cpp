#include "driftless/op_control/arm/ArmOperator.hpp"

namespace driftless::op_control::arm {
ArmOperator::ArmOperator(const std::shared_ptr<io::IController>& controller,
                         const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ArmOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  bool go_score_up{m_controller->getDigital(
      profile->getDigitalControlMapping(EControl::LIFT_RAISE))};
  bool go_score_down{m_controller->getDigital(
      profile->getDigitalControlMapping(EControl::LIFT_LOWER))};
  bool go_load{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::ARM_GO_LOAD))};
  bool reset{m_controller->getNewDigital(
      profile->getDigitalControlMapping(EControl::ARM_RESET))};

  if (go_score_up) {
    if (getLiftHeight() > 29.0f) {
      goScoreHigh();
    } else {
      goScoreLow();
    }
  } else if (go_score_down) {
    goScoreLow();
  } else if (go_load) {
    goLoad();
  } else if (reset) {
    resetArm();
  }
}

inline void ArmOperator::goLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::commands::arm::ArmGoLoadCommand{});
}

inline void ArmOperator::goScoreLow() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::commands::arm::ArmGoScoreLowCommand{});
}

inline void ArmOperator::goScoreHigh() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::commands::arm::ArmGoScoreHighCommand{});
}

inline void ArmOperator::resetArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::commands::arm::ArmResetCommand{});
}

inline float ArmOperator::getLiftHeight() {
  return *static_cast<float*>(
      m_robot->getState(robot::subsystems::ESubsystem::LIFT,
                        robot::subsystems::ESubsystemState::LIFT_GET_HEIGHT));
}
}  // namespace driftless::op_control::arm
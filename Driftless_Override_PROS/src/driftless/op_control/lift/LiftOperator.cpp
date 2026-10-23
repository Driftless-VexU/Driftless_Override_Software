#include "driftless/op_control/lift/LiftOperator.hpp"

namespace driftless::op_control::lift {
LiftOperator::LiftOperator(const std::shared_ptr<io::IController>& controller,
                           const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void LiftOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital lift_raise(
      profile->getDigitalControlMapping(op_control::EControl::LIFT_RAISE));
  EControllerDigital lift_lower(
    profile->getDigitalControlMapping(op_control::EControl::LIFT_LOWER));

    static constexpr float LIFT_INCREMENT = 3.0f;
    static constexpr float LIFT_MAX_HEIGHT = 25.0f;
    static constexpr float LIFT_TOLERANCE = 0.5f;
    static constexpr float LIFT_MIN_HEIGHT = 0.0f;

  float current_height = getLiftHeight();
  if (m_controller->getDigital(lift_raise)) {
    if(current_height > target_height - LIFT_TOLERANCE && target_height != LIFT_MAX_HEIGHT){
      target_height += LIFT_INCREMENT;
      if(target_height > LIFT_MAX_HEIGHT){
        target_height = LIFT_MAX_HEIGHT;
      }
      setLiftHeight(target_height);
    }
  } else if(m_controller->getDigital(lift_lower)) {
    if(current_height < target_height + LIFT_TOLERANCE && target_height != LIFT_MIN_HEIGHT){
      target_height -= LIFT_INCREMENT;
      if(target_height < LIFT_MIN_HEIGHT){
        target_height = LIFT_MIN_HEIGHT;
      }
      setLiftHeight(target_height);
    }   
  }
}

float LiftOperator::getLiftHeight() {
  return *static_cast<float*>(
      m_robot->getState(robot::subsystems::ESubsystem::LIFT,
                        robot::subsystems::ESubsystemState::LIFT_GET_HEIGHT));
}

void LiftOperator::setLiftHeight(float height) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::LIFT,
                       robot::commands::lift::SetLiftHeightCommand{height});
}
}  // namespace driftless::op_control::lift
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
  EControllerDigital lift_load{
      profile->getDigitalControlMapping(op_control::EControl::ARM_GO_LOAD)};

  static constexpr float LIFT_INCREMENT = 3.0f;
  static constexpr float LIFT_MAX_HEIGHT = 29.5f;
  static constexpr float LIFT_TOLERANCE = 1.0f;
  static constexpr float LIFT_MIN_HEIGHT = 0.0f;

  float current_height = getLiftHeight();
  if (getArmState() == 7 || getArmState() == 6) {
    if (m_controller->getDigital(lift_raise)) {
      if (current_height > target_height - LIFT_TOLERANCE &&
          target_height != LIFT_MAX_HEIGHT) {
        target_height += LIFT_INCREMENT;
        if (target_height > LIFT_MAX_HEIGHT) {
          target_height = LIFT_MAX_HEIGHT;
        }
        setLiftHeight(target_height);
      }
    } else if (m_controller->getDigital(lift_lower)) {
      if (current_height < target_height + LIFT_TOLERANCE &&
          target_height != LIFT_MIN_HEIGHT) {
        target_height -= LIFT_INCREMENT;
        if (target_height < LIFT_MIN_HEIGHT) {
          target_height = LIFT_MIN_HEIGHT;
        }
      }
      setLiftHeight(target_height);
    }
  }

  if (m_controller->getDigital(lift_load)) {
    is_load_pressed = true;
    target_height = 6.0;
    setLiftHeight(target_height);
  } else {
    if (is_load_pressed && getArmState() == 3) {
      target_height = 1.25f;
      setLiftHeight(target_height);
      is_load_pressed = false;
    }
  }
}

float LiftOperator::getLiftHeight() {
  void* state =
      m_robot->getState(robot::subsystems::ESubsystem::LIFT,
                        robot::subsystems::ESubsystemState::LIFT_GET_HEIGHT);
  if (!state) {
    throw std::runtime_error(
        "LiftOperator: Lift subsystem state LIFT_GET_HEIGHT is null");
  }
  return *static_cast<float*>(state);
}

void LiftOperator::setLiftHeight(float height) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::LIFT,
                       robot::commands::lift::SetLiftHeightCommand{height});
}

uint8_t LiftOperator::getArmState() {
  return *static_cast<uint8_t*>(m_robot->getState(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemState::ARM_GET_CURRENT_STATE));
}
}  // namespace driftless::op_control::lift
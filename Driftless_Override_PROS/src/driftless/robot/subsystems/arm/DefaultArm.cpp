#include "driftless/robot/subsystems/arm/DefaultArm.hpp"

#include "driftless/robot/commands/lift/SetLiftHeightCommand.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/arm/DefaultArmBuilder.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

namespace driftless::robot::subsystems::arm {
void DefaultArm::init() {
  m_elbow_motors.init();
  m_wrist_motors.init();

  m_elbow_pid.reset();
  m_wrist_pid.reset();
}

void DefaultArm::run() {
  if (m_task) {
    m_task->start(DefaultArm::taskLoop, this);
  }
}

void DefaultArm::reset() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_do_initialize = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToLoadPosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_target_position_state != EArmState::LOAD) {
    m_elbow_pid.reset();
    m_wrist_pid.reset();
  }
  m_target_position_state = EArmState::LOAD;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToScanPosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_target_position_state != EArmState::SCAN) {
    m_elbow_pid.reset();
    m_wrist_pid.reset();
  }
  m_target_position_state = EArmState::SCAN;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToScoreLowPosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_target_position_state != EArmState::SCORE_LOW) {
    m_elbow_pid.reset();
    m_wrist_pid.reset();
  }
  m_target_position_state = EArmState::SCORE_LOW;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToScoreHighPosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_target_position_state != EArmState::SCORE_HIGH) {
    m_elbow_pid.reset();
    m_wrist_pid.reset();
  }
  m_target_position_state = EArmState::SCORE_HIGH;

  if (m_mutex) {
    m_mutex->give();
  }
}

EArmState DefaultArm::getCurrentPositionState() {
  if (m_mutex) {
    m_mutex->take();
  }

  EArmState current_state{m_current_position_state};

  if (m_mutex) {
    m_mutex->give();
  }

  return current_state;
}

void DefaultArm::taskLoop(void* params) {
  DefaultArm* arm{static_cast<DefaultArm*>(params)};
  while (true) {
    arm->taskUpdate();
  }
}

DefaultArm::DefaultArm(DefaultArmBuilder&& builder)
    : m_elbow_motors{std::move(builder.m_elbow_motors)},
      m_wrist_motors{std::move(builder.m_wrist_motors)},
      m_elbow_pid{builder.m_elbow_pid},
      m_wrist_pid{builder.m_wrist_pid},
      m_arm_targets{builder.m_arm_targets},
      m_task{std::move(builder.m_task)},
      m_mutex{std::move(builder.m_mutex)},
      m_delayer{std::move(builder.m_delayer)},
      m_robot{builder.m_robot},
      m_elbow_position_tolerance{builder.m_elbow_position_tolerance},
      m_wrist_position_tolerance{builder.m_wrist_position_tolerance} {}

void DefaultArm::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();

    if (m_do_initialize) {
      m_wrist_motors.setVoltage(-6.0);
      m_elbow_motors.setVoltage(-6.0);
      uint16_t timeout_counter{0};
      m_delayer->delay(100);
      timeout_counter += 100;
      do {
        timeout_counter += TASK_DELAY;
        m_delayer->delay(TASK_DELAY);
      } while ((std::abs(m_wrist_motors.getAngularVelocity()) > 0.1 ||
                std::abs(m_elbow_motors.getAngularVelocity()) > 0.1) &&
               timeout_counter < TASK_TIMEOUT);

      if (timeout_counter < TASK_TIMEOUT) {
        m_wrist_motors.setPosition(0.0);
        m_elbow_motors.setPosition(-0.25 * 2 * M_PI);
        m_is_initialized = true;
        m_current_position_state = EArmState::INITIALIZED;
      }

      m_wrist_motors.setVoltage(0);
      m_elbow_motors.setVoltage(0);
      m_do_initialize = false;
    }
  }
  if (m_is_initialized) {
    switch (m_target_position_state) {
      case EArmState::START:
      case EArmState::INITIALIZED:
      case EArmState::LOAD_PRE:
      case EArmState::LOAD_POST:
        // these cases should never be targets - Do nothing
        break;
      case EArmState::LOAD:
        handleLoadState();
        break;
      case EArmState::SCAN:
        handleScanState();
        break;
      case EArmState::SCORE_LOW:
        handleScoreLowState();
        break;
      case EArmState::SCORE_HIGH:
        handleScoreHighState();
        break;
    }

    m_elbow_motors.setVoltage(m_elbow_pid.getControlValue(
        m_elbow_motors.getPosition(), m_elbow_target_position, 12.0));
    m_wrist_motors.setVoltage(m_wrist_pid.getControlValue(
        m_wrist_motors.getPosition(), m_wrist_target_position, 12.0));
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

inline void DefaultArm::handleLoadState() {
  if (isClawFlipped()) {
    flipClaw(false);
  } else if (m_current_position_state == EArmState::LOAD_PRE) {
    m_elbow_target_position = m_arm_targets.elbow_load_position;
    m_wrist_target_position = m_arm_targets.wrist_load_position;

    float elbow_error{
        std::abs(static_cast<float>(m_elbow_motors.getPosition()) -
                 m_elbow_target_position)};
    float wrist_error{
        std::abs(static_cast<float>(m_wrist_motors.getPosition()) -
                 m_wrist_target_position)};

    if ((elbow_error < m_elbow_position_tolerance) &&
        (wrist_error < m_wrist_position_tolerance) && !isClawFlipped()) {
      // arm is in position and can be lowered into the loader
      m_current_position_state = EArmState::LOAD;
      setLiftHeight(m_arm_targets.lift_load_height);
    }

  } else if (m_current_position_state != EArmState::LOAD) {
    if (getLiftHeight() <= m_arm_targets.lift_load_transition_height) {
      setLiftHeight(m_arm_targets.lift_load_transition_height);
    } else {
      // lift is high enough to move arm into staging area
      m_current_position_state = EArmState::LOAD_PRE;
    }
  }
}

inline void DefaultArm::handleScanState() {
  if (m_current_position_state == EArmState::LOAD) {
    if (getLiftHeight() >= m_arm_targets.lift_load_transition_height) {
      m_current_position_state = EArmState::LOAD_POST;
    }

  } else if (m_current_position_state != EArmState::SCAN) {
    if (m_current_position_state == EArmState::LOAD_POST) {
      m_elbow_target_position = m_arm_targets.elbow_score_low_position;
    }

    m_wrist_target_position = m_arm_targets.wrist_scan_position;

    float elbow_error{
        std::abs(static_cast<float>(m_elbow_motors.getPosition()) -
                 m_elbow_target_position)};
    float wrist_error{
        std::abs(static_cast<float>(m_wrist_motors.getPosition()) -
                 m_wrist_target_position)};

    if ((elbow_error < m_elbow_position_tolerance) &&
        (wrist_error < m_wrist_position_tolerance)) {
      m_current_position_state = EArmState::SCAN;
    }
  }
}

inline void DefaultArm::handleScoreLowState() {
  if (m_current_position_state == EArmState::LOAD) {
    setLiftHeight(m_arm_targets.lift_load_transition_height + 1);
    if (getLiftHeight() >= m_arm_targets.lift_load_transition_height) {
      m_current_position_state = EArmState::LOAD_POST;
    }

  } else {
    m_elbow_target_position = m_arm_targets.elbow_score_low_position;

    float elbow_rad =
        m_elbow_motors.getPosition() * m_elbow_motor_rad_to_actual_rad;

    m_wrist_target_position =
        (elbow_rad + (M_PI / 6.0)) / m_wrist_motor_rad_to_actual_rad;

    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7, "Elbow angle: %7.2f",
                        elbow_rad * 180 / M_PI);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8,
                        "target wrist pos: %7.2f", m_wrist_target_position);
    pros::screen::print(
        pros::E_TEXT_MEDIUM_CENTER, 9, "target elbow angle: %7.2f",
        m_elbow_target_position * m_elbow_motor_rad_to_actual_rad * 180 / M_PI);

    float elbow_error{
        std::abs(static_cast<float>(m_elbow_motors.getPosition()) -
                 m_elbow_target_position)};
    float wrist_error{
        std::abs(static_cast<float>(m_wrist_motors.getPosition()) -
                 m_wrist_target_position)};

    if ((elbow_error < m_elbow_position_tolerance) &&
        (wrist_error < m_wrist_position_tolerance)) {
      m_current_position_state = EArmState::SCORE_LOW;
    }
  }
}

inline void DefaultArm::handleScoreHighState() {
  if (m_current_position_state == EArmState::LOAD) {
    if (getLiftHeight() >= m_arm_targets.lift_load_transition_height) {
      m_current_position_state = EArmState::LOAD_POST;
    }

  } else {
    m_elbow_target_position = m_arm_targets.elbow_score_high_position;

    float elbow_rad =
        m_elbow_motors.getPosition() * m_elbow_motor_rad_to_actual_rad;

    m_wrist_target_position =
        (elbow_rad + (M_PI / 6.0)) / m_wrist_motor_rad_to_actual_rad;

    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7, "Elbow angle: %7.2f",
                        elbow_rad * 180 / M_PI);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8,
                        "target wrist pos: %7.2f", m_wrist_target_position);
    pros::screen::print(
        pros::E_TEXT_MEDIUM_CENTER, 9, "target elbow angle: %7.2f",
        m_elbow_target_position * m_elbow_motor_rad_to_actual_rad * 180 / M_PI);

    float elbow_error{
        std::abs(static_cast<float>(m_elbow_motors.getPosition()) -
                 m_elbow_target_position)};
    float wrist_error{
        std::abs(static_cast<float>(m_wrist_motors.getPosition()) -
                 m_wrist_target_position)};

    if ((elbow_error < m_elbow_position_tolerance) &&
        (wrist_error < m_wrist_position_tolerance)) {
      m_current_position_state = EArmState::SCORE_HIGH;
    }
  }
}

inline float DefaultArm::getLiftHeight() {
  float out{0.0f};

  if (auto robot = m_robot.lock()) {
    out = *static_cast<float*>(
        robot->getState(ESubsystem::LIFT, ESubsystemState::LIFT_GET_HEIGHT));
  }
  return out;
}

inline void DefaultArm::setLiftHeight(float height) {
  if (auto robot = m_robot.lock()) {
    robot->sendCommand(ESubsystem::LIFT,
                       commands::lift::SetLiftHeightCommand{height});
  }
}

inline bool DefaultArm::isClawFlipped() {
  // TODO: implement when claw is ready
  bool out{true};
  if (auto robot = m_robot.lock()) {
    out = !(*static_cast<bool*>(
        robot->getState(ESubsystem::CLAW, ESubsystemState::CLAW_IS_UPRIGHT)));
  }

  return out;
}

inline void DefaultArm::flipClaw(bool orientation) {
  if (auto robot = m_robot.lock()) {
    robot->sendCommand(ESubsystem::CLAW,
                       commands::claw::SetClawUpDownCommand{!orientation});
  }
}

}  // namespace driftless::robot::subsystems::arm
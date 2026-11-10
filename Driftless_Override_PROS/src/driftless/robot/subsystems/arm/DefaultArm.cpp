#include "driftless/robot/subsystems/arm/DefaultArm.hpp"

#include "driftless/robot/commands/lift/SetLiftHeightCommand.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/arm/DefaultArmBuilder.hpp"

namespace driftless::robot::subsystems::arm {
void DefaultArm::init() {
  m_elbow_motors.init();
  m_wrist_motors.init();
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

  m_wrist_motors.setVoltage(-5.0);
  m_elbow_motors.setVoltage(-5.0);
  uint16_t timeout_counter{0};
  do {
    timeout_counter += TASK_DELAY;
    m_delayer->delay(TASK_DELAY);
  } while (std::abs(m_wrist_motors.getAngularVelocity()) > 0.1 &&
           std::abs(m_elbow_motors.getAngularVelocity()) > 0.1 &&
           timeout_counter < TASK_TIMEOUT);

  if (timeout_counter < TASK_TIMEOUT) {
    m_wrist_motors.setPosition(0.0);
    m_elbow_motors.setPosition(0.0);
    m_is_initialized = true;
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToLoadPosition() {
  if (m_mutex) {
    m_mutex->take();
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

  m_target_position_state = EArmState::SCAN;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DefaultArm::goToScoreLowPosition() {
  if (m_mutex) {
    m_mutex->take();
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
  }
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

  if (m_is_initialized) {
    m_elbow_motors.setVoltage(m_elbow_pid.getControlValue(
        m_elbow_motors.getPosition(), m_elbow_target_position));
    m_wrist_motors.setVoltage(m_wrist_pid.getControlValue(
        m_wrist_motors.getPosition(), m_wrist_target_position));
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

inline void DefaultArm::handleLoadState() {
  if (m_current_position_state == EArmState::LOAD_PRE) {
    m_elbow_target_position = m_arm_targets.elbow_load_position;
    m_wrist_target_position = m_arm_targets.wrist_load_position;

    float elbow_error{
        std::abs(static_cast<float>(m_elbow_motors.getPosition()) -
                 m_elbow_target_position)};
    float wrist_error{
        std::abs(static_cast<float>(m_wrist_motors.getPosition()) -
                 m_wrist_target_position)};

    if (isClawFlipped()) {
      flipClaw(false);
    }

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
    setLiftHeight(m_arm_targets.lift_load_transition_height + 1);

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

  } else if (m_current_position_state != EArmState::SCORE_LOW) {
    m_elbow_target_position = m_arm_targets.elbow_score_low_position;
    m_wrist_target_position = m_arm_targets.wrist_score_low_position;

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
    setLiftHeight(m_arm_targets.lift_load_transition_height + 1);

    if (getLiftHeight() >= m_arm_targets.lift_load_transition_height) {
      m_current_position_state = EArmState::LOAD_POST;
    }

  } else if (m_current_position_state != EArmState::SCORE_HIGH) {
    m_elbow_target_position = m_arm_targets.elbow_score_high_position;
    m_wrist_target_position = m_arm_targets.wrist_score_high_position;

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
  return false; /*
   *static_cast<bool>(
       m_robot.getState(ESubsystem::CLAW, ESubsystemState::CLAW_IS_FLIPPED));
       */
}

inline void DefaultArm::flipClaw(bool orientation) {
  /* m_robot.sendCommand(ESubsystem::CLAW, commands::claw::CLAW_FLIP{}); */
}

}  // namespace driftless::robot::subsystems::arm
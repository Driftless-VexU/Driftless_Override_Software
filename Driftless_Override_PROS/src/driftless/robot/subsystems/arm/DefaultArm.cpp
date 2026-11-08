#include "driftless/robot/subsystems/arm/DefaultArm.hpp"
// #include "driftless/robot/subsystems/arm/DefaultArmBuilder.hpp"

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

void DefaultArm::resetWrist() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_wrist_motors.setVoltage(-5.0);
  uint16_t timeout_counter{0};
  do {
    timeout_counter += TASK_DELAY;
    m_delayer->delay(TASK_DELAY);
  } while (std::abs(m_wrist_motors.getAngularVelocity()) > 0.1 &&
           timeout_counter < TASK_TIMEOUT);

  if (timeout_counter < TASK_TIMEOUT) {
    m_wrist_motors.setPosition(0.0);
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

}  // namespace driftless::robot::subsystems::arm
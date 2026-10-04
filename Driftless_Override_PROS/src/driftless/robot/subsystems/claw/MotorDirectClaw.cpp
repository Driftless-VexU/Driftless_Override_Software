#include "driftless/robot/subsystems/claw/MotorDirectClaw.hpp"

#include <cmath>

#include "driftless/robot/subsystems/claw/MotorDirectClawBuilder.hpp"

namespace driftless::robot::subsystems::claw {

void MotorDirectClaw::init() {}

void MotorDirectClaw::run() { m_task->start(&MotorDirectClaw::taskLoop, this); }

void MotorDirectClaw::goToSetPosition(const EClawPositionName pos) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_target_position = pos;
  m_target_elbow_rotation =
      m_claw_positions[static_cast<int>(m_target_position)].m_elbow_rotation;
  m_target_wrist_rotation =
      m_claw_positions[static_cast<int>(m_target_wrist_rotation)]
          .m_wrist_rotation;

  if (m_mutex) {
    m_mutex->give();
  }
}

void MotorDirectClaw::setElbowRotation(float target) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_target_position = EClawPositionName::MANUAL;
  m_target_elbow_rotation =
      std::max(0.0f, std::min(static_cast<float>(M_PI), target)) /
      m_motor_to_elbow_rotations;

  if (m_mutex) {
    m_mutex->give();
  }
}

void MotorDirectClaw::setWristRotation(float target) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_target_position = EClawPositionName::MANUAL;
  m_target_wrist_rotation =
      std::max(0.0f, std::min(static_cast<float>(M_PI), target)) /
      m_motor_to_wrist_rotations;

  if (m_mutex) {
    m_mutex->give();
  }
}

void MotorDirectClaw::flipClaw() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_is_flipped = !m_is_flipped;
  m_target_flip_rotation = (m_is_flipped ? 0.0f : static_cast<float>(M_PI)) /
                           m_motor_to_flip_rotations;

  if (m_mutex) {
    m_mutex->give();
  }
}

float MotorDirectClaw::getElbowRotation() {
  return m_elbow_motors.getPosition() * m_motor_to_elbow_rotations;
}

float MotorDirectClaw::getWristRotation() {
  return m_wrist_motors.getPosition() * m_motor_to_wrist_rotations;
}

bool MotorDirectClaw::isClawFlipped() { return m_is_flipped; }

void MotorDirectClaw::taskLoop(void* params) {
  MotorDirectClaw* claw{static_cast<MotorDirectClaw*>(params)};

  while (1) {
    claw->taskUpdate();
  }
}

MotorDirectClaw::MotorDirectClaw(MotorDirectClawBuilder&& builder)
    : m_elbow_motors{std::move(builder.m_elbow_motors)},
      m_wrist_motors{std::move(builder.m_wrist_motors)},
      m_flip_motors{std::move(builder.m_flip_motors)},
      m_claw_positions{builder.m_claw_positions},
      m_elbow_pid{builder.m_elbow_pid},
      m_wrist_pid{builder.m_wrist_pid},
      m_flip_pid{builder.m_flip_pid},
      m_task{std::move(builder.m_task)},
      m_mutex{std::move(builder.m_mutex)},
      m_delayer{std::move(builder.m_delayer)},
      m_motor_to_elbow_rotations{builder.m_motor_to_elbow_rotations},
      m_motor_to_wrist_rotations{builder.m_motor_to_wrist_rotations},
      m_motor_to_flip_rotations{builder.m_motor_to_flip_rotations} {}

void MotorDirectClaw::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_elbow_motors.setVoltage(m_elbow_pid.getControlValue(
      m_elbow_motors.getPosition(), m_target_elbow_rotation));
  m_wrist_motors.setVoltage(m_wrist_pid.getControlValue(
      m_wrist_motors.getPosition(), m_target_wrist_rotation));
  m_flip_motors.setVoltage(m_flip_pid.getControlValue(
      m_flip_motors.getPosition(), m_target_flip_rotation));

  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}
}  // namespace driftless::robot::subsystems::claw
#include "driftless/robot/subsystems/lift/DirectContinuousLift.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/lift/DirectContinuousLiftBuilder.hpp"
namespace driftless::robot::subsystems::lift {
void DirectContinuousLift::init() {}

void DirectContinuousLift::run() { m_task->start(taskLoop, this); }

void DirectContinuousLift::setHeight(float height) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_height = height;

  if (m_mutex) {
    m_mutex->give();
  }
}

float DirectContinuousLift::getHeight() {
    return m_motors.getPosition() * m_rotations_to_inches;
}

void DirectContinuousLift::taskLoop(void* args) {
  DirectContinuousLift* lift{static_cast<DirectContinuousLift*>(args)};

  while (1) {
    lift->taskUpdate();
  }
}

void DirectContinuousLift::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }
  float voltage = m_pid.getControlValue(m_motors.getPosition() * m_rotations_to_inches, m_height);
  m_motors.setVoltage(voltage);
  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}

DirectContinuousLift::DirectContinuousLift(DirectContinuousLiftBuilder&& builder)
    : m_motors(std::move(builder.m_motors)),
        m_task(std::move(builder.m_task)),
        m_delayer(std::move(builder.m_delayer)),
        m_mutex(std::move(builder.m_mutex)) {}
}  // namespace driftless::robot::subsystems::lift
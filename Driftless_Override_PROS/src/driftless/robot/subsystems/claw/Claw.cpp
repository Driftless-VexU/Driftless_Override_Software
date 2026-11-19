#include "driftless/robot/subsystems/claw/Claw.hpp"

#include <cmath>

#include "driftless/robot/subsystems/claw/ClawBuilder.hpp"

namespace driftless::robot::subsystems::claw {
void Claw::init() {
  m_pivot_motor.init();
  m_pivot_motor.setVoltage(3.5);
  m_delayer->delay(600);
  m_pivot_motor.setCurrentLimit(2.5);
  m_pivot_motor.setPosition(0);
  m_target_pivot_position = 0;
  m_pivot_voltage = 3.0;
}

void Claw::run() { m_task->start(taskLoop, this); }

void Claw::setOpenClosed(bool is_open) {
  if (m_mutex) {
    m_mutex->take();
  }

  if (is_open) {
    m_piston->extend();
  } else {
    m_piston->retract();
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void Claw::setUpDown(bool is_up) {
  if (m_mutex) {
    m_mutex->take();
  }

  if (is_up) {
    m_pivot_voltage = -12.0;
  } else {
    m_pivot_voltage = 12.0;
  }
  time_flipping = 0;

  if (m_mutex) {
    m_mutex->give();
  }
}

void Claw::toggleUpDown() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_pivot_voltage = 12.0 * -(m_pivot_voltage / std::abs(m_pivot_voltage));
  m_is_up = !m_is_up;
  time_flipping = 0;

  if (m_mutex) {
    m_mutex->give();
  }
}

void Claw::toggleOpenClosed() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_piston->toggleState();

  if (m_mutex) {
    m_mutex->give();
  }
}

bool Claw::isClawUp() { return m_is_up; }

Claw::Claw(ClawBuilder&& builder)
    : m_pivot_motor{std::move(builder.m_pivot_motor)},
      m_pivot_pid{builder.m_pivot_pid},
      m_piston{std::move(builder.m_piston)},
      m_task{std::move(builder.m_task)},
      m_mutex{std::move(builder.m_mutex)},
      m_delayer{std::move(builder.m_delayer)},
      m_pivot_flipped_position{builder.m_pivot_flipped_position},
      m_pivot_position_tolerance{builder.m_pivot_position_tolerance} {}

void Claw::taskLoop(void* params) {
  Claw* claw{static_cast<Claw*>(params)};

  while (1) {
    claw->taskUpdate();
  }
}

void Claw::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (time_flipping > 1000) {
    if (std::abs(m_pivot_motor.getAngularVelocity()) < 0.1 &&
        std::abs(m_pivot_voltage) > 4.0) {
      m_pivot_voltage /= 4.0;
    }
  } else {
    time_flipping += TASK_DELAY;
  }
  m_pivot_motor.setVoltage(m_pivot_voltage);

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}
}  // namespace driftless::robot::subsystems::claw

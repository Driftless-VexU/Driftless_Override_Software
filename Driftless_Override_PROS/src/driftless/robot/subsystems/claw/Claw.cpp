#include "driftless/robot/subsystems/claw/Claw.hpp"

#include <cmath>

#include "driftless/robot/subsystems/claw/ClawBuilder.hpp"
namespace driftless::robot::subsystems::claw {
void Claw::init() {}

void Claw::run() {}

void Claw::setOpenClosed(bool isOpen) {
  if (isOpen) {
    m_piston->extend();
  } else {
    m_piston->retract();
  }
}

void Claw::setUpDown(bool is_up) {
  m_target_pivot_position = is_up * m_pivot_flipped_position;
}

void Claw::toggleOpenClosed() { m_piston->toggleState(); }

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

  if (std::abs(m_pivot_motor.getPosition() - m_target_pivot_position) <=
      m_pivot_position_tolerance) {
    m_pivot_motor.setVoltage(0.0);

    if (m_target_pivot_position == 0.0f) {
      m_is_up = false;
    } else {
      m_is_up = true;
    }
  } else {
    float pivot_voltage = m_pivot_pid.getControlValue(
        m_pivot_motor.getPosition(), m_target_pivot_position);
    m_pivot_motor.setVoltage(pivot_voltage);
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}
}  // namespace driftless::robot::subsystems::claw

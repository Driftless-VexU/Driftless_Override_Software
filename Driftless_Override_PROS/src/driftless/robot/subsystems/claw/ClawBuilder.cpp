#include "driftless/robot/subsystems/claw/ClawBuilder.hpp"

namespace driftless::robot::subsystems::claw {
ClawBuilder&& ClawBuilder::withPiston(std::unique_ptr<io::IPiston> piston) && {
  m_piston = std::move(piston);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withMotor(std::unique_ptr<io::IMotor> motor) && {
  m_pivot_motor.addMotor(std::move(motor));
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withPivotPID(control::PID pivot_pid) && {
  m_pivot_pid = pivot_pid;
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withTask(std::unique_ptr<rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withMutex(std::unique_ptr<rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withPivotFlippedPosition(
    float pivot_flipped_position) && {
  m_pivot_flipped_position = pivot_flipped_position;
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withPivotPositionTolerance(
    float pivot_position_tolerance) && {
  m_pivot_position_tolerance = pivot_position_tolerance;
  return std::move(*this);
}

std::unique_ptr<Claw> ClawBuilder::buildUnique() && {
  return std::unique_ptr<Claw>{new Claw(std::move(*this))};
}
Claw ClawBuilder::build() && { return Claw(std::move(*this)); }

}  // namespace driftless::robot::subsystems::claw
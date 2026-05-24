#include "driftless/control/motion/PIDHolonomicTurnBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withRotationalPID(
    PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return *this;
}

PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withTargetTolerance(
    double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return *this;
}

PIDHolonomicTurnBuilder& PIDHolonomicTurnBuilder::withTargetVelocity(
    double target_velocity) {
  m_target_velocity = target_velocity;
  return *this;
}

PIDHolonomicTurn PIDHolonomicTurnBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return PIDHolonomicTurn{std::move(*this)};
}

std::unique_ptr<PIDHolonomicTurn> PIDHolonomicTurnBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDHolonomicTurn>{
      new PIDHolonomicTurn{std::move(*this)}};
}
}  // namespace driftless::control::motion
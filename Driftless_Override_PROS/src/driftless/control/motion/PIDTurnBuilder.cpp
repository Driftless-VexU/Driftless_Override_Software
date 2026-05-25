#include "driftless/control/motion/PIDTurnBuilder.hpp"

namespace driftless {
namespace control {
namespace motion {
PIDTurnBuilder& PIDTurnBuilder::withDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDTurnBuilder& PIDTurnBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex> mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDTurnBuilder& PIDTurnBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask> task) {
  m_task = std::move(task);
  return *this;
}

PIDTurnBuilder& PIDTurnBuilder::withRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return *this;
}

PIDTurnBuilder& PIDTurnBuilder::withTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return *this;
}

PIDTurnBuilder& PIDTurnBuilder::withTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
  return *this;
}

PIDTurn PIDTurnBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return PIDTurn{std::move(*this)};
}

std::unique_ptr<PIDTurn> PIDTurnBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDTurn>{new PIDTurn{std::move(*this)}};
}
}  // namespace motion
}  // namespace control
}  // namespace driftless
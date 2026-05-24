#include "driftless/control/motion/PIDGoToPointBuilder.hpp"

namespace driftless {
namespace control {
namespace motion {
PIDGoToPointBuilder& PIDGoToPointBuilder::withDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withLinearPID(PID linear_pid) {
  m_linear_pid = linear_pid;
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withRotationalPID(
    PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withTargetTolerance(
    double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return *this;
}

PIDGoToPointBuilder& PIDGoToPointBuilder::withTargetVelocity(
    double target_velocity) {
  m_target_velocity = target_velocity;
  return *this;
}

PIDGoToPoint PIDGoToPointBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return PIDGoToPoint{std::move(*this)};
}

std::unique_ptr<PIDGoToPoint> PIDGoToPointBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDGoToPoint>{new PIDGoToPoint{std::move(*this)}};
}
}  // namespace motion
}  // namespace control
}  // namespace driftless
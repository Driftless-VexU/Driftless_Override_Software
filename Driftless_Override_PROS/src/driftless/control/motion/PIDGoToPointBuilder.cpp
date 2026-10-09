#include "driftless/control/motion/PIDGoToPointBuilder.hpp"

namespace driftless {
namespace control {
namespace motion {
PIDGoToPointBuilder&& PIDGoToPointBuilder::withDelayer(
    std::unique_ptr<driftless::rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withLinearPID(PID linear_pid) && {
  m_linear_pid = std::move(linear_pid);
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withRotationalPID(
    PID rotational_pid) && {
  m_rotational_pid = std::move(rotational_pid);
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withTargetTolerance(
    double target_tolerance) && {
  m_target_tolerance = target_tolerance;
  return std::move(*this);
}

PIDGoToPointBuilder&& PIDGoToPointBuilder::withTargetVelocity(
    double target_velocity) && {
  m_target_velocity = target_velocity;
  return std::move(*this);
}

PIDGoToPoint PIDGoToPointBuilder::build() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return PIDGoToPoint{std::move(*this)};
}

std::unique_ptr<PIDGoToPoint> PIDGoToPointBuilder::buildUnique() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDGoToPoint>{new PIDGoToPoint{std::move(*this)}};
}
}  // namespace motion
}  // namespace control
}  // namespace driftless
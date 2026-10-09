#include "driftless/control/motion/PIDDriveStraightBuilder.hpp"

namespace driftless {
namespace control {
namespace motion {
PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withDelayer(
    std::unique_ptr<driftless::rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withLinearPID(
    PID linear_pid) && {
  m_linear_pid = std::move(linear_pid);
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withRotationalPID(
    PID rotational_pid) && {
  m_rotational_pid = std::move(rotational_pid);
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withTargetTolerance(
    double target_tolerance) && {
  m_target_tolerance = target_tolerance;
  return std::move(*this);
}

PIDDriveStraightBuilder&& PIDDriveStraightBuilder::withTargetVelocity(
    double target_velocity) && {
  m_target_velocity = target_velocity;
  return std::move(*this);
}

PIDDriveStraight PIDDriveStraightBuilder::build() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for PIDDriveStraightBuilder");
  }

  return PIDDriveStraight{std::move(*this)};
}

std::unique_ptr<PIDDriveStraight> PIDDriveStraightBuilder::buildUnique() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for PIDDriveStraightBuilder");
  }

  return std::unique_ptr<PIDDriveStraight>{
      new PIDDriveStraight{std::move(*this)}};
}
}  // namespace motion
}  // namespace control
}  // namespace driftless
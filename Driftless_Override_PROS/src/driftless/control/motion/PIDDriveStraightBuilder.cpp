#include "driftless/control/motion/PIDDriveStraightBuilder.hpp"

namespace driftless {
namespace control {
namespace motion {
PIDDriveStraightBuilder& PIDDriveStraightBuilder::withDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withLinearPID(
    PID linear_pid) {
  m_linear_pid = linear_pid;
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withRotationalPID(
    PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withTargetTolerance(
    double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return *this;
}

PIDDriveStraightBuilder& PIDDriveStraightBuilder::withTargetVelocity(
    double target_velocity) {
  m_target_velocity = target_velocity;
  return *this;
}

PIDDriveStraight PIDDriveStraightBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for PIDDriveStraightBuilder");
  }

  return {std::move(*this)};
}

std::unique_ptr<PIDDriveStraight> PIDDriveStraightBuilder::buildUnique() {
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
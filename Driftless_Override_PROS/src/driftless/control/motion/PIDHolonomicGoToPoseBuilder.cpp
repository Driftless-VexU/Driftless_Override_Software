#include "driftless/control/motion/PIDHolonomicGoToPoseBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withXPID(
    const PID& pid) {
  m_x_pid = pid;
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withYPID(
    const PID& pid) {
  m_y_pid = pid;
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withRotationalPID(
    const PID& pid) {
  m_rotational_pid = pid;
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withDistanceTolerance(
    double tolerance) {
  m_distance_tolerance = tolerance;
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withVelocityTolerance(
    double tolerance) {
  m_velocity_tolerance = tolerance;
  return *this;
}

PIDHolonomicGoToPoseBuilder& PIDHolonomicGoToPoseBuilder::withAngularTolerance(
    double tolerance) {
  m_angular_tolerance = tolerance;
  return *this;
}

PIDHolonomicGoToPose PIDHolonomicGoToPoseBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return {std::move(*this)};
}

std::unique_ptr<PIDHolonomicGoToPose>
PIDHolonomicGoToPoseBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDHolonomicGoToPose>{
      new PIDHolonomicGoToPose{std::move(*this)}};
}
}  // namespace driftless::control::motion
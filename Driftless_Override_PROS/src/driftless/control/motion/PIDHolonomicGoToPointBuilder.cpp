#include "driftless/control/motion/PIDHolonomicGoToPointBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicGoToPointBuilder& PIDHolonomicGoToPointBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDHolonomicGoToPointBuilder& PIDHolonomicGoToPointBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDHolonomicGoToPointBuilder& PIDHolonomicGoToPointBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDHolonomicGoToPointBuilder& PIDHolonomicGoToPointBuilder::withXPID(
    const PID& pid) {
  m_x_pid = pid;
  return *this;
}

PIDHolonomicGoToPointBuilder& PIDHolonomicGoToPointBuilder::withYPID(
    const PID& pid) {
  m_y_pid = pid;
  return *this;
}

PIDHolonomicGoToPointBuilder&
PIDHolonomicGoToPointBuilder::withDistanceTolerance(double tolerance) {
  m_distance_tolerance = tolerance;
  return *this;
}

PIDHolonomicGoToPointBuilder&
PIDHolonomicGoToPointBuilder::withVelocityTolerance(double tolerance) {
  m_velocity_tolerance = tolerance;
  return *this;
}

PIDHolonomicGoToPoint PIDHolonomicGoToPointBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return PIDHolonomicGoToPoint{std::move(*this)};
}

std::unique_ptr<PIDHolonomicGoToPoint>
PIDHolonomicGoToPointBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDPathFollowerBuilder");
  }

  return std::unique_ptr<PIDHolonomicGoToPoint>{
      new PIDHolonomicGoToPoint{std::move(*this)}};
}
}  // namespace driftless::control::motion
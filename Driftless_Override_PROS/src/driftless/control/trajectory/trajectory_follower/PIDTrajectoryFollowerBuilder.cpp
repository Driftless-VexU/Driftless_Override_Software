#include "driftless/control/trajectory/trajectory_follower/PIDTrajectoryFollowerBuilder.hpp"

namespace driftless::control::trajectory::trajectory_follower {
PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withClock(
    const std::unique_ptr<rtos::IClock>& clock) {
  m_clock = clock->clone();
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withXPID(
    PID& x_pid) {
  m_x_pid = x_pid;
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withYPID(
    PID& y_pid) {
  m_y_pid = y_pid;
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withThetaPID(
    PID& theta_pid) {
  m_theta_pid = theta_pid;
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withTargetTolerance(
    double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return *this;
}

PIDTrajectoryFollowerBuilder& PIDTrajectoryFollowerBuilder::withTargetVelocity(
    double target_velocity) {
  m_target_velocity = target_velocity;
  return *this;
}

PIDTrajectoryFollower PIDTrajectoryFollowerBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task || !m_clock) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDTrajectoryFollowerBuilder");
  }

  return PIDTrajectoryFollower{std::move(*this)};
}

std::unique_ptr<PIDTrajectoryFollower>
PIDTrajectoryFollowerBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task || !m_clock) {
    throw std::runtime_error(
        "One or more RTOS components not set in PIDTrajectoryFollowerBuilder");
  }
  
  return std::unique_ptr<PIDTrajectoryFollower>{
      new PIDTrajectoryFollower{std::move(*this)}};
}
}  // namespace driftless::control::trajectory::trajectory_follower
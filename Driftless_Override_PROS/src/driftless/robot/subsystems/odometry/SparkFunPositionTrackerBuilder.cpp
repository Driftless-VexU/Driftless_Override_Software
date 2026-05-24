#include "driftless/robot/subsystems/odometry/SparkFunPositionTrackerBuilder.hpp"

namespace driftless::robot::subsystems::odometry {
SparkFunPositionTrackerBuilder& SparkFunPositionTrackerBuilder::withClock(
    std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return *this;
}

SparkFunPositionTrackerBuilder& SparkFunPositionTrackerBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return *this;
}

SparkFunPositionTrackerBuilder& SparkFunPositionTrackerBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

SparkFunPositionTrackerBuilder& SparkFunPositionTrackerBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

SparkFunPositionTrackerBuilder&
SparkFunPositionTrackerBuilder::withLocalXOffset(double local_x_offset) {
  m_local_x_offset = local_x_offset;
  return *this;
}

SparkFunPositionTrackerBuilder&
SparkFunPositionTrackerBuilder::withLocalYOffset(double local_y_offset) {
  m_local_y_offset = local_y_offset;
  return *this;
}

SparkFunPositionTrackerBuilder&
SparkFunPositionTrackerBuilder::withLocalThetaOffset(
    double local_theta_offset) {
  m_local_theta_offset = local_theta_offset;
  return *this;
}

SparkFunPositionTracker SparkFunPositionTrackerBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for "
        "SparkFunPositionTrackerBuilder");
  }

  return SparkFunPositionTracker{std::move(*this)};
}

std::unique_ptr<SparkFunPositionTracker>
SparkFunPositionTrackerBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for "
        "SparkFunPositionTrackerBuilder");
  }

  return std::unique_ptr<SparkFunPositionTracker>{
      new SparkFunPositionTracker{std::move(*this)}};
}
}  // namespace driftless::robot::subsystems::odometry
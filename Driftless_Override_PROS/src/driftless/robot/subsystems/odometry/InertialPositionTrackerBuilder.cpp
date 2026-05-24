#include "driftless/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
InertialPositionTrackerBuilder& InertialPositionTrackerBuilder::withClock(
    std::unique_ptr<driftless::rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return *this;
}

InertialPositionTrackerBuilder& InertialPositionTrackerBuilder::withDelayer(
    std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return *this;
}

InertialPositionTrackerBuilder& InertialPositionTrackerBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return *this;
}

InertialPositionTrackerBuilder& InertialPositionTrackerBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return *this;
}

InertialPositionTrackerBuilder&
InertialPositionTrackerBuilder::withInertialSensor(
    std::unique_ptr<driftless::io::IInertialSensor>& inertial_sensor) {
  m_inertial_sensor = std::move(inertial_sensor);
  return *this;
}

InertialPositionTrackerBuilder&
InertialPositionTrackerBuilder::withLinearDistanceTracker(
    std::unique_ptr<driftless::io::IDistanceTracker>& linear_distance_tracker) {
  m_linear_distance_tracker = std::move(linear_distance_tracker);
  return *this;
}

InertialPositionTrackerBuilder&
InertialPositionTrackerBuilder::withLinearDistanceTrackerOffset(
    double linear_distance_tracker_offset) {
  m_linear_distance_tracker_offset = linear_distance_tracker_offset;
  return *this;
}

InertialPositionTrackerBuilder&
InertialPositionTrackerBuilder::withStrafeDistanceTracker(
    std::unique_ptr<driftless::io::IDistanceTracker>& strafe_distance_tracker) {
  m_strafe_distance_tracker = std::move(strafe_distance_tracker);
  return *this;
}

InertialPositionTrackerBuilder&
InertialPositionTrackerBuilder::withStrafeDistanceTrackerOffset(
    double strafe_distance_tracker_offset) {
  m_strafe_distance_tracker_offset = strafe_distance_tracker_offset;
  return *this;
}

InertialPositionTracker InertialPositionTrackerBuilder::build() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for "
        "InertialPositionTrackerBuilder");
  }

  return InertialPositionTracker{std::move(*this)};
}

std::unique_ptr<InertialPositionTracker>
InertialPositionTrackerBuilder::buildUnique() {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for InertialPositionTrackerBuilder");
  }

  return std::unique_ptr<InertialPositionTracker>{
      new InertialPositionTracker{std::move(*this)}};
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
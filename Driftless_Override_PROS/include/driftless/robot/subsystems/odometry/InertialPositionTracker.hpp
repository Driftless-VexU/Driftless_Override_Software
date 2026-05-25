#ifndef __INERTIAL_POSITION_TRACKER_HPP__
#define __INERTIAL_POSITION_TRACKER_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IInertialSensor.hpp"
#include "driftless/robot/subsystems/odometry/IPositionTracker.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

class InertialPositionTrackerBuilder;

/// @brief Class for tracking the position of the robot using inertial sensors
/// @author Matthew Backman
class InertialPositionTracker : public IPositionTracker {
  friend class InertialPositionTrackerBuilder;

 private:
  // the delay between task loops
  static constexpr uint8_t TASK_DELAY{10};

  // conversion factor between milliseconds and seconds
  static constexpr double MS_TO_SECONDS{1.0 / 1000.0};

  // task loop to update provided tracker
  static void taskLoop(void* params);

  // clock object
  std::unique_ptr<driftless::rtos::IClock> m_clock{};

  // task delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // task for updates
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // inertial sensor to track rotation
  std::unique_ptr<driftless::io::IInertialSensor> m_inertial_sensor{};

  // distance tracker on the linear side of the robot
  std::unique_ptr<driftless::io::IDistanceTracker> m_linear_distance_tracker{};

  // the offset, from the linear side of the robot, of the linear distance
  // tracker
  double m_linear_distance_tracker_offset{};

  // distance tracker on the strafe side of the robot
  std::unique_ptr<driftless::io::IDistanceTracker> m_strafe_distance_tracker{};

  // the offset, from the linear side of the robot, of the strafe distance
  // tracker
  double m_strafe_distance_tracker_offset{};

  // the position on the field
  Position m_position{};

  // the heading offset
  double m_heading_offset{};

  // the latest heading value
  double last_heading{};

  // the latest distance recorded on the linear tracking wheel
  double last_linear_distance{};

  // the latest distance recorded on the strafe tracking wheel
  double last_strafe_distance{};

  // the system time of the latest update
  uint8_t last_time{};

  explicit InertialPositionTracker(InertialPositionTrackerBuilder&& builder);

  // run all updates in the system
  void taskUpdate();

  // update the position of the system
  void updatePosition();

 public:
  InertialPositionTracker(const InertialPositionTracker&) = delete;

  /// @brief Move constructor
  /// @param other __InertialPositionTracker&&__ The position tracker to move
  InertialPositionTracker(InertialPositionTracker&& other) = default;

  /// @brief Initializes the system
  void init() override;

  /// @brief Runs the system
  void run() override;

  /// @brief Sets the position of the system
  /// @param position __Position__ The position to set
  void setPosition(Position position) override;

  /// @brief Gets the position of the system
  /// @return __Position__ The current position of the system
  Position getPosition() override;

  /// @brief Sets the x position
  /// @param x __double__ The x position to set
  void setX(double x) override;

  /// @brief Sets the y position
  /// @param y __double__ The y position to set
  void setY(double y) override;

  /// @brief Sets the angular offset of the system
  /// @param theta __double__ The angular offset to set
  void setTheta(double theta) override;
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif
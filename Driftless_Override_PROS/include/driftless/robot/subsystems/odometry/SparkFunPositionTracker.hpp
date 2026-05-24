#ifndef __SPARK_FUN_POSITION_TRACKER_HPP__
#define __SPARK_FUN_POSITION_TRACKER_HPP__

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "driftless/io/ISerialDevice.hpp"
#include "driftless/robot/subsystems/odometry/IPositionTracker.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

class SparkFunPositionTrackerBuilder;

/// @brief Class representing a position tracker using the SparkFun Odometry
/// sensor
class SparkFunPositionTracker : public IPositionTracker {
  friend class SparkFunPositionTrackerBuilder;

 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Task loop to update the given position tracker
  /// @param params __void*__ The position tracker being updated
  static void taskLoop(void* params);

  std::unique_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  double m_local_x_offset{};
  double m_local_y_offset{};
  double m_local_theta_offset{};

  double global_x_offset{};
  double global_y_offset{};
  double global_theta_offset{};

  Position current_position{};

  uint32_t latest_time{};

  explicit SparkFunPositionTracker(SparkFunPositionTrackerBuilder&& builder);

  /// @brief Runs all updates related to the position tracker
  void taskUpdate();

  /// @brief Updates the internal position
  void updatePosition();

  /// @brief Gathers the raw position from the odometry sensor
  /// @return __Position__ The position the sensor thinks it's at
  Position fetchRawPosition();

  /// @brief Sends the local offsets to the arduino
  void sendLocalOffset();

 public:
  /// @brief deleted copy constructor
  /// @param other Object to copy
  SparkFunPositionTracker(const SparkFunPositionTracker& other) = delete;

  /// @brief Move constructor
  /// @param other __SparkFunPositionTracker&&__ The position tracker to move
  SparkFunPositionTracker(SparkFunPositionTracker&& other) = default;

  /// @brief Initializes the position tracker
  void init() override;

  /// @brief Runs the position tracker
  void run() override;

  /// @brief Sets the global position of the position tracker
  /// @param position __Position__ The new position
  void setPosition(Position position) override;

  /// @brief Gets the current position of the position tracker
  /// @return __Position__ The current position
  Position getPosition() override;

  /// @brief Sets the x position of the position tracker
  /// @param x __double__ The new x position
  void setX(double x) override;

  /// @brief Sets the y position of the position tracker
  /// @param y __double__ The new y position
  void setY(double y) override;

  /// @brief Sets the heading of the position tracker
  /// @param theta __double__ The new heading
  void setTheta(double theta) override;
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
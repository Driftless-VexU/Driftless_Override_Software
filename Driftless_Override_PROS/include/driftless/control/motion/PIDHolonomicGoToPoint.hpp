#ifndef __PID_HOLONOMIC_GO_TO_POINT_HPP__
#define __PID_HOLONOMIC_GO_TO_POINT_HPP__

#include <cmath>
#include <cstdint>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/IGoToPoint.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

class PIDHolonomicGoToPointBuilder;

/// @brief Class representing a go to point algorithm using PID
/// @author Matthew Backman
class PIDHolonomicGoToPoint : public IGoToPoint {
  friend class PIDHolonomicGoToPointBuilder;

 private:
  // the task delay
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly updates the given goToPoint algorithm
  /// @param params __void*__ The goToPoint algorithm to update
  static void taskLoop(void* params);

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::shared_ptr<robot::Robot> m_robot{};

  PID m_x_pid{};

  PID m_y_pid{};

  double m_max_velocity{};

  double m_distance_tolerance{};

  double m_velocity_tolerance{};

  Point m_target_point{};

  bool m_target_reached{};

  bool m_paused{true};

  /// @brief Constructs a new PIDHolonomicGoToPoint from a builder
  /// @param builder __PIDHolonomicGoToPointBuilder&&__ The builder to construct
  /// from
  PIDHolonomicGoToPoint(PIDHolonomicGoToPointBuilder&& builder);

  /// @brief Sets the drive motion vector of the robot
  /// @param x_velocity __double__ The velocity in the x direction
  /// @param y_velocity __double__ The velocity in the y direction
  /// @param angular_velocity __double__ The angular velocity
  void setDriveMotionVector(double x_velocity, double y_velocity,
                            double angular_velocity);

  /// @brief Gets the current position of the robot
  /// @return The current position of the robot
  robot::subsystems::odometry::Position getPosition();

  /// @brief Updates the velocity of the robot
  /// @param x_distance __double__ The distance to the target in the x
  /// direction
  /// @param y_distance __double__ The distance to the target in the y
  /// direction
  /// @param current_heading __double__ The current heading of the robot
  void updateVelocity(double x_distance, double y_distance,
                      double current_heading);

  /// @brief Runs all instance specific updates
  void taskUpdate();

 public:
  /// @brief Copies another PIDHolonomicGoToPoint
  /// @param other __const PIDHolonomicGoToPoint&__ The PIDHolonomicGoToPoint
  /// being copied
  PIDHolonomicGoToPoint(const PIDHolonomicGoToPoint& other) = default;

  /// @brief Moves a PIDHolonomicGoToPoint
  /// @param other __PIDHolonomicGoToPoint&&__ The PIDHolonomicGoToPoint being
  /// moved
  PIDHolonomicGoToPoint(PIDHolonomicGoToPoint&& other) = default;
  
  /// @brief Initializes the control
  void init() override;

  /// @brief Runs the control
  void run() override;

  /// @brief Pauses the control
  void pause() override;

  /// @brief Resumes the control
  void resume() override;

  /// @brief Sets the target point for the robot to go to
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max velocity for motion
  /// @param point __Point__ The target point
  void goToPoint(const std::shared_ptr<robot::Robot>& robot, double velocity,
                 Point point) override;

  /// @brief Updates the max velocity for motion
  /// @param velocity __double__ The max velocity for motion
  void setVelocity(double velocity) override;

  /// @brief Checks if the target point has been reached
  /// @return True if the target point has been reached, false otherwise
  bool targetReached() override;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless

#endif
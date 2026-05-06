#ifndef __PID_TRAJECTORY_FOLLOWER_HPP__
#define __PID_TRAJECTORY_FOLLOWER_HPP__

#include "driftless/control/PID.hpp"
#include "driftless/control/trajectory/trajectory_follower/ITrajectoryFollower.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Namespace for trajectory following algorithms
/// @author Matthew Backman
namespace trajectory_follower {

class PIDTrajectoryFollowerBuilder;

/// @brief Control algorithm to follow a generated trajectory using PID
class PIDTrajectoryFollower : public ITrajectoryFollower {

 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Repeatedly updates the given trajectory follower
  /// @param params __void*__ The trajectory follower to update
  static void taskLoop(void* params);

  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  std::unique_ptr<driftless::rtos::ITask> m_task{};

  std::unique_ptr<driftless::rtos::IClock> m_clock{};

  PID m_x_pid{};

  PID m_y_pid{};

  PID m_theta_pid{};

  double m_target_tolerance{};

  double m_target_velocity{};

  std::shared_ptr<driftless::robot::Robot> m_robot{};

  std::vector<TrajectoryPoint> m_trajectory{};

  uint32_t m_elapsed_time{};

  uint32_t m_last_time{};

  bool m_paused{};

  bool m_target_reached{true};

  /// @brief Updates the trajectory follower
  void taskUpdate();

  /// @brief Sets the drive motion vector of the robot
  /// @param x_velocity __double__ The x velocity
  /// @param y_velocity __double__ The y velocity
  /// @param angular_velocity __double__ The angular velocity
  void setDriveMotionVector(double x_velocity, double y_velocity,
                            double angular_velocity);

  /// @brief Gets the current position of the robot
  /// @return __Position__ The current position of the robot
  driftless::robot::subsystems::odometry::Position getRobotPosition();

  /// @brief Calculates the distance to the target
  /// @param position __const Position__ The current position
  /// @return __double__ The distance to the target
  double calculateDistanceToTarget(
      const driftless::robot::subsystems::odometry::Position& position);

  /// @brief Updates the target velocity of the robot
  /// @param position __const Position__ The current position
  /// @param target_point __const TrajectoryPoint__ The target point
  void updateVelocity(
      const driftless::robot::subsystems::odometry::Position& position,
      const TrajectoryPoint& target_point);

 public:
  /// @brief Constructs a new PIDTrajectoryFollower object with the provided
  /// builder
  /// @param builder __PIDTrajectoryFollowerBuilder&&__ The builder containing
  /// the parameters for the trajectory follower
  PIDTrajectoryFollower(PIDTrajectoryFollowerBuilder&& builder);

  /// @brief Initializes the trajectory follower
  void init() override;

  /// @brief Runs the trajectory follower
  void run() override;

  /// @brief Pauses the trajectory follower
  void pause() override;

  /// @brief Resumes the trajectory follower
  void resume() override;

  /// @brief Follows a given trajectory
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param trajectory __const std::vector<TrajectoryPoint>&__ The trajectory
  /// to follow
  void followTrajectory(
      const std::shared_ptr<driftless::robot::Robot>& robot,
      const std::vector<TrajectoryPoint>& trajectory) override;

  /// @brief Determines if the target has been reached
  /// @return __bool__ True if the target has been reached, false otherwise
  bool targetReached() override;
};
}  // namespace trajectory_follower
}  // namespace trajectory
}  // namespace control
}  // namespace driftless

#endif
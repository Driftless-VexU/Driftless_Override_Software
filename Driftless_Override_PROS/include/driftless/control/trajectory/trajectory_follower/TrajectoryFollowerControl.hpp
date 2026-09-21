#ifndef __TRAJECTORY_FOLLOWER_CONTROL_HPP__
#define __TRAJECTORY_FOLLOWER_CONTROL_HPP__

#include "driftless/control/AControl.hpp"
#include "driftless/control/trajectory/trajectory_follower/ITrajectoryFollower.hpp"

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

/// @brief Wrapper class for trajectory followers to adapt to control system
class TrajectoryFollowerControl : public AControl {
 private:
  std::unique_ptr<ITrajectoryFollower> m_trajectory_follower{};

  /// @brief Handles the follow trajectory command
  /// @param cmd __FollowTrajectoryCommand&__ The command to handle
  void handleCommand(
      const commands::trajectory::FollowTrajectoryCommand& cmd) const;

  /// @brief Handles any command with no explicit handler
  /// @throws std::invalid_argument for all commands with no explicit handler
  /// @param cmd __auto&__ The command to handle
  void handleCommand(const auto& cmd) const;

 public:
  /// @brief Constructs a new TrajectoryFollowerControl
  /// @param trajectory_follower __std::unique_ptr<ITrajectoryFollower>&__ The
  /// trajectory follower to wrap
  TrajectoryFollowerControl(
      std::unique_ptr<ITrajectoryFollower>& trajectory_follower);

  /// @brief Initializes the TrajectoryFollowerControl
  void init() override;

  /// @brief Runs the TrajectoryFollowerControl
  void run() override;

  /// @brief Pauses the TrajectoryFollowerControl
  void pause() override;

  /// @brief Resumes the TrajectoryFollowerControl
  void resume() override;

  /// @brief Sends a command to the trajectory follower
  /// @param command __commands::Command&__ The command to send
  void command(const commands::Command& command) override;

  /// @brief Gets a state of the trajectory follower
  /// @param state_name __EControlState__ The state to gather
  /// @return __void*__ The state of the trajectory follower
  void* state(EControlState state_name) override;
};
}  // namespace trajectory_follower
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif
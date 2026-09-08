#ifndef __PATH_FOLLOWER_CONTROL_HPP__
#define __PATH_FOLLOWER_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/path/PIDPathFollowerBuilder.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Class representing the path follower control
/// @author Matthew Backman
class PathFollowerControl : public driftless::control::AControl {
 private:
  // path follower object
  std::unique_ptr<driftless::control::path::IPathFollower> m_path_follower{};

  /// @brief Handles the follow path command
  /// @param cmd __path::FollowPathCommand&__ The command to handle
  void handleCommand(const commands::path::FollowPathCommand& cmd);

  /// @brief Handles the set linear velocity command
  /// @param cmd __path::SetLinearVelocityCommand&__ The command to handle
  void handleCommand(const commands::SetLinearVelocityCommand& cmd);

  /// @brief Handles any command with no explicit handler
  /// @throws std::invalid_argument for all commands with no explicit handler
  /// @param cmd __auto&__ The command to handle
  void handleCommand(const auto& cmd) const;

 public:
  /// @brief Constructs a new path follower control
  /// @param path_follower __std::unique_ptr<IPathFollower>& The path follower
  /// to control
  PathFollowerControl(
      std::unique_ptr<driftless::control::path::IPathFollower>& path_follower);

  /// @brief Initializes the path follower control
  void init() override;

  /// @brief Runs the path follower control
  void run() override;

  /// @brief Pauses the path follower control
  void pause() override;

  /// @brief Resumes the path follower control
  void resume() override;

  /// @brief Sends a command to the path follower
  /// @param command __commands::Command&__ The command to send
  void command(const commands::Command& command) override;

  /// @brief Gets a state of the path follower
  /// @param state_name __EControlState__ The state to get
  /// @return __void*__ The state of the path follower
  void* state(EControlState state_name) override;
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif
#include "driftless/control/trajectory/trajectory_follower/TrajectoryFollowerControl.hpp"

namespace driftless::control::trajectory::trajectory_follower {
void TrajectoryFollowerControl::handleCommand(
    const commands::trajectory::FollowTrajectoryCommand& cmd) const {
  m_trajectory_follower->followTrajectory(cmd.m_robot, cmd.m_trajectory);
}

void TrajectoryFollowerControl::handleCommand(const auto& cmd) const {
  throw std::invalid_argument(
      "TrajectoryFollowerControl: No handler for command type " +
      std::string{typeid(cmd).name()});
}

TrajectoryFollowerControl::TrajectoryFollowerControl(
    std::unique_ptr<ITrajectoryFollower>& trajectory_follower)
    : m_trajectory_follower{std::move(trajectory_follower)},
      AControl(EControl::TRAJECTORY_FOLLOWER) {}

void TrajectoryFollowerControl::init() { m_trajectory_follower->init(); }

void TrajectoryFollowerControl::run() { m_trajectory_follower->run(); }

void TrajectoryFollowerControl::pause() { m_trajectory_follower->pause(); }

void TrajectoryFollowerControl::resume() { m_trajectory_follower->resume(); }

void TrajectoryFollowerControl::command(const commands::Command& command) {
  std::visit([this](const auto& cmd) { handleCommand(cmd); }, command);
}

void* TrajectoryFollowerControl::state(EControlState state_name) {
  void* result{nullptr};

  switch (state_name) {
    case (EControlState::TRAJECTORY_FOLLOWER_TARGET_REACHED): {
      result = new bool(m_trajectory_follower->targetReached());
    }
  }

  return result;
}
}  // namespace driftless::control::trajectory::trajectory_follower
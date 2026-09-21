#include "driftless/control/path/PathFollowerControl.hpp"

namespace driftless {
namespace control {
namespace path {
void PathFollowerControl::handleCommand(
    const commands::path::FollowPathCommand& cmd) {
  m_path_follower->followPath(cmd.m_robot, cmd.m_path, cmd.m_velocity);
}

void PathFollowerControl::handleCommand(
    const commands::SetLinearVelocityCommand& cmd) {
  m_path_follower->setVelocity(cmd.m_linear_velocity);
}

void PathFollowerControl::handleCommand(const auto& cmd) const {
  throw std::invalid_argument(
      "PathFollowerControl: No handler for command type " +
      std::string{typeid(cmd).name()});
}

PathFollowerControl::PathFollowerControl(
    std::unique_ptr<driftless::control::path::IPathFollower>& path_follower)
    : AControl{EControl::PATH_FOLLOWER},
      m_path_follower{std::move(path_follower)} {}

void PathFollowerControl::init() {
  if (m_path_follower) {
    m_path_follower->init();
  }
}

void PathFollowerControl::run() {
  if (m_path_follower) {
    m_path_follower->run();
  }
}

void PathFollowerControl::pause() {
  if (m_path_follower) {
    m_path_follower->pause();
  }
}

void PathFollowerControl::resume() {
  if (m_path_follower) {
    m_path_follower->resume();
  }
}

void PathFollowerControl::command(const commands::Command& command) {
  std::visit([this](const auto& cmd) { handleCommand(cmd); }, command);
}

void* PathFollowerControl::state(EControlState state_name) {
  void* result{nullptr};
  if (state_name == EControlState::PATH_FOLLOWER_TARGET_REACHED) {
    result = new bool(m_path_follower->targetReached());
  }
  return result;
}
}  // namespace path
}  // namespace control
}  // namespace driftless
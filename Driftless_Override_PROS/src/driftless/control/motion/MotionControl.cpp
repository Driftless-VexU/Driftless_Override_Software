#include "driftless/control/motion/MotionControl.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace control {
namespace motion {
void MotionControl::switchMotionType(EMotionType motion_type) {
  if (m_motion_type != motion_type) {
    pause();
    m_motion_type = motion_type;
  }
}

void MotionControl::handleCommand(
    const commands::motion::DriveStraightCommand& cmd) {
  switchMotionType(EMotionType::DRIVE_STRAIGHT);
  m_drive_straight->driveStraight(cmd.m_robot, cmd.m_velocity, cmd.m_distance,
                                  cmd.m_theta);
}

void MotionControl::handleCommand(
    const commands::motion::GoToPointCommand& cmd) {
  switchMotionType(EMotionType::GO_TO_POINT);
  m_go_to_point->goToPoint(cmd.m_robot, cmd.m_velocity,
                           Point{cmd.m_x, cmd.m_y});
}

void MotionControl::handleCommand(
    const commands::motion::GoToPoseCommand& cmd) {
  switchMotionType(EMotionType::GO_TO_POSE);
  m_go_to_pose->goToPose(cmd.m_robot, cmd.m_linear_velocity,
                         cmd.m_angular_velocity, cmd.m_linear_acceleration,
                         Point{cmd.m_x, cmd.m_y, cmd.m_theta});
}

void MotionControl::handleCommand(
    const commands::motion::TurnToAngleCommand& cmd) {
  switchMotionType(EMotionType::TURN);
  m_turn->turnToAngle(cmd.m_robot, cmd.m_angular_velocity, cmd.m_theta,
                      cmd.m_direction);
}

void MotionControl::handleCommand(
    const commands::motion::TurnToPointCommand& cmd) {
  switchMotionType(EMotionType::TURN);
  m_turn->turnToPoint(cmd.m_robot, cmd.m_angular_velocity,
                      Point{cmd.m_x_pos, cmd.m_y_pos}, cmd.m_direction);
}

void MotionControl::handleCommand(
    const commands::SetLinearVelocityCommand& cmd) const {
  m_drive_straight->setVelocity(cmd.m_linear_velocity);
  m_go_to_point->setVelocity(cmd.m_linear_velocity);
  m_go_to_pose->setVelocity(cmd.m_linear_velocity);
}

void MotionControl::handleCommand(
    const commands::SetAngularVelocityCommand& cmd) const {
  m_go_to_pose->setAngularVelocity(cmd.m_angular_velocity);
}

void MotionControl::handleCommand(const auto& cmd) const {
  throw std::invalid_argument("MotionControl: No handler for command type " +
                              std::string{typeid(cmd).name()});
}

MotionControl::MotionControl(
    std::unique_ptr<driftless::control::motion::IDriveStraight>& drive_straight,
    std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
    std::unique_ptr<driftless::control::motion::IGoToPose>& go_to_pose,
    std::unique_ptr<driftless::control::motion::ITurn>& turn)
    : AControl{EControl::MOTION},
      m_drive_straight{std::move(drive_straight)},
      m_go_to_point{std::move(go_to_point)},
      m_go_to_pose{std::move(go_to_pose)},
      m_turn{std::move(turn)} {}

void MotionControl::init() {
  m_drive_straight->init();
  m_go_to_point->init();
  m_go_to_pose->init();
  m_turn->init();
}

void MotionControl::run() {
  m_drive_straight->run();
  m_go_to_point->run();
  m_go_to_pose->run();
  m_turn->run();
}

void MotionControl::pause() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->pause();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->pause();
      break;
    case EMotionType::GO_TO_POSE:
      m_go_to_pose->pause();
      break;
    case EMotionType::TURN:
      m_turn->pause();
      break;
  }
}

void MotionControl::resume() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->resume();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->resume();
      break;
    case EMotionType::GO_TO_POSE:
      m_go_to_pose->resume();
    case EMotionType::TURN:
      m_turn->resume();
      break;
  }
}

void MotionControl::command(const commands::Command& command) {
  std::visit([this](auto&& cmd) { handleCommand(cmd); }, command);
}

void* MotionControl::state(EControlState state_name) {
  void* result{};
  if (state_name == EControlState::DRIVE_STRAIGHT_TARGET_REACHED) {
    result = new bool{m_drive_straight->targetReached()};
  } else if (state_name == EControlState::GO_TO_POINT_TARGET_REACHED) {
    result = new bool{m_go_to_point->targetReached()};
  } else if (state_name == EControlState::GO_TO_POSE_TARGET_REACHED) {
    result = new bool{m_go_to_pose->targetReached()};
  } else if (state_name == EControlState::TURN_TARGET_REACHED) {
    result = new bool{m_turn->targetReached()};
  }

  return result;
}
}  // namespace motion
}  // namespace control
}  // namespace driftless
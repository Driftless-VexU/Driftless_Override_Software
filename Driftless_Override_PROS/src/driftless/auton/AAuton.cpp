#include "driftless/auton/AAuton.hpp"

namespace driftless::auton {
uint32_t AAuton::getTime() {
  uint32_t time{};
  if (m_clock) {
    time = m_clock->getTime();
  }

  return time;
}

void AAuton::delay(uint32_t delay_time) {
  if (m_delayer) {
    m_delayer->delay(delay_time);
  }
}

void AAuton::delayUntil(uint32_t time) {
  double current_time{};

  if (m_clock) {
    current_time = m_clock->getTime();
  }

  if (m_delayer && current_time < time) {
    m_delayer->delayUntil(time);
  }
}

robot::subsystems::odometry::Position AAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

void AAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::commands::odometry::SetPositionCommand{x, y, theta});
}

void AAuton::followTrajectory(
    std::vector<control::trajectory::TrajectoryPoint>& trajectory) {
  m_control_system->sendCommand(
      control::EControl::TRAJECTORY_FOLLOWER,
      control::commands::trajectory::FollowTrajectoryCommand{m_robot,
                                                             trajectory});
}

bool AAuton::trajectoryTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::TRAJECTORY_FOLLOWER,
      control::EControlState::TRAJECTORY_FOLLOWER_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForTrajectory(control::trajectory::TrajectoryPoint& endpoint,
                               double tolerance, uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     endpoint.m_x, endpoint.m_y)};

  while (getTime() < start_time + timeout && !trajectoryTargetReached() &&
         std::abs(distance_to_target) > tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  endpoint.m_x, endpoint.m_y);
    delay(LOOP_DELAY);
  }
}

void AAuton::goToPoint(control::Point target_point, double target_velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::motion::GoToPointCommand{
          m_robot, target_velocity, target_point.getX(), target_point.getY()});
}

bool AAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForGoToPoint(control::Point target_point, double tolerance,
                              uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     target_point.getX(), target_point.getY())};

  while (getTime() < start_time + timeout && !goToPointTargetReached() &&
         std::abs(distance_to_target) > tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  target_point.getX(), target_point.getY());
    delay(LOOP_DELAY);
  }
}

void AAuton::setMotionLinearVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::SetLinearVelocityCommand{velocity});
}

void AAuton::setMotionAngularVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::SetAngularVelocityCommand{velocity});
}

void AAuton::goToPose(control::Point target_point, double target_velocity,
                      double target_angular_velocity,
                      double linear_acceleration) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::motion::GoToPoseCommand{
          m_robot, target_velocity, target_angular_velocity,
          linear_acceleration, target_point.getX(), target_point.getY(),
          target_point.getTheta()});
}

bool AAuton::goToPoseTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POSE_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForGoToPose(control::Point target_point,
                             double position_tolerance, uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     target_point.getX(), target_point.getY())};

  while (getTime() < start_time + timeout && !goToPoseTargetReached() &&
         std::abs(distance_to_target) > position_tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  target_point.getX(), target_point.getY());
    delay(LOOP_DELAY);
  }
}

void AAuton::turnToPoint(control::Point target_point, double target_velocity,
                         control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::motion::TurnToPointCommand{
          m_robot, target_velocity, target_point.getX(), target_point.getY(),
          direction});
}

void AAuton::turnToHeading(double heading, double target_velocity,
                           control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::commands::motion::TurnToAngleCommand{m_robot, target_velocity,
                                                    heading, direction});
}

bool AAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForTurnToPoint(control::Point target_point, double tolerance,
                                uint32_t timeout) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double angle_difference{
      bindRadians(angle(current_position.x, current_position.y,
                        target_point.getX(), target_point.getY()) -
                  current_position.theta)};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(angle_difference) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    angle_difference =
        bindRadians(angle(current_position.x, current_position.y,
                          target_point.getX(), target_point.getY()) -
                    current_position.theta);
    m_delayer->delay(LOOP_DELAY);
  }
}

void AAuton::waitForTurnToAngle(double heading, double tolerance,
                                uint32_t timeout) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(bindRadians(current_position.theta - heading)) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    m_delayer->delay(LOOP_DELAY);
  }
}

void AAuton::stopMotion() {
  m_control_system->pause();

  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::commands::holonomic_drive_train::SetMotionVectorCommand{
          0.0, 0.0, 0.0, true});
}

AAuton::AAuton(std::string name) : m_name{name} {}

std::string AAuton::getName() { return m_name; }
}  // namespace driftless::auton
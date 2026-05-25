#include "driftless/control/motion/PIDHolonomicGoToPoint.hpp"
#include "driftless/control/motion/PIDHolonomicGoToPointBuilder.hpp"

namespace driftless::control::motion {
void PIDHolonomicGoToPoint::taskLoop(void* params) {
  PIDHolonomicGoToPoint* go_to_point{
      static_cast<PIDHolonomicGoToPoint*>(params)};

  while (true) {
    go_to_point->taskUpdate();
  }
}

PIDHolonomicGoToPoint::PIDHolonomicGoToPoint(PIDHolonomicGoToPointBuilder&& builder)
    : m_delayer(std::move(builder.m_delayer)),
      m_mutex(std::move(builder.m_mutex)),
      m_task(std::move(builder.m_task)),
      m_x_pid(std::move(builder.m_x_pid)),
      m_y_pid(std::move(builder.m_y_pid)),
      m_distance_tolerance(builder.m_distance_tolerance),
      m_velocity_tolerance(builder.m_velocity_tolerance) {}

void PIDHolonomicGoToPoint::setDriveMotionVector(double x_velocity,
                                                 double y_velocity,
                                                 double angular_velocity) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::subsystems::ESubsystemCommand::
          HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
      robot::subsystems::holonomic_drive_train::HolonomicMotionVector{
          x_velocity, y_velocity, angular_velocity});
}

robot::subsystems::odometry::Position PIDHolonomicGoToPoint::getPosition() {
  return *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
}

void PIDHolonomicGoToPoint::updateVelocity(double x_distance, double y_distance,
                                           double current_heading) {
  double x_velocity{m_x_pid.getControlValue(0, x_distance)};
  double y_velocity{m_y_pid.getControlValue(0, y_distance)};

  double velocity_scalar{1.0};
  double velocity{std::sqrt(x_velocity * x_velocity + y_velocity * y_velocity)};
  if (velocity > m_max_velocity) {
    velocity_scalar = m_max_velocity / velocity;
  }
  x_velocity *= velocity_scalar;
  y_velocity *= velocity_scalar;

  double out_x = x_velocity * std::sin(current_heading) -
                 y_velocity * std::cos(current_heading);
  double out_y = x_velocity * std::cos(current_heading) +
                 y_velocity * std::sin(current_heading);

  setDriveMotionVector(out_x, out_y, 0);
}

void PIDHolonomicGoToPoint::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_paused && !m_target_reached) {
    robot::subsystems::odometry::Position position{getPosition()};

    double x_distance{m_target_point.getX() - position.x};
    double y_distance{m_target_point.getY() - position.y};
    double distance_to_target{
        std::sqrt(x_distance * x_distance + y_distance * y_distance)};
    double velocity{
        std::sqrt(position.xV * position.xV + position.yV * position.yV)};

    if (distance_to_target < m_distance_tolerance &&
        velocity < m_velocity_tolerance) {
      m_target_reached = true;
      setDriveMotionVector(0, 0, 0);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8, "Target Reached");
    } else {
      updateVelocity(x_distance, y_distance, position.theta);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDHolonomicGoToPoint::init() {
  m_x_pid.reset();
  m_y_pid.reset();
}

void PIDHolonomicGoToPoint::run() {
  if (m_task) {
    m_task->start(&PIDHolonomicGoToPoint::taskLoop, this);
  }
}

void PIDHolonomicGoToPoint::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::goToPoint(
    const std::shared_ptr<robot::Robot>& robot, double velocity, Point point) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  m_max_velocity = velocity;
  m_target_point = point;
  m_target_reached = false;
  m_paused = false;

  m_x_pid.reset();
  m_y_pid.reset();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_max_velocity = velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDHolonomicGoToPoint::targetReached() { return m_target_reached; }
}  // namespace driftless::control::motion
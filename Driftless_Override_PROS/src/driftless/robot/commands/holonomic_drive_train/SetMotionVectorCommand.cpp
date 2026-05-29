#include "driftless/robot/commands/holonomic_drive_train/SetMotionVectorCommand.hpp"

namespace driftless::robot::commands::holonomic_drive_train {
SetMotionVectorCommand::SetMotionVectorCommand(double x_velocity,
                                               double y_velocity,
                                               double angular_velocity)
    : m_x_velocity(x_velocity),
      m_y_velocity(y_velocity),
      m_angular_velocity(angular_velocity) {}

double SetMotionVectorCommand::getXVelocity() const { return m_x_velocity; }

double SetMotionVectorCommand::getYVelocity() const { return m_y_velocity; }

double SetMotionVectorCommand::getAngularVelocity() const {
  return m_angular_velocity;
}
}  // namespace driftless::robot::commands::holonomic_drive_train
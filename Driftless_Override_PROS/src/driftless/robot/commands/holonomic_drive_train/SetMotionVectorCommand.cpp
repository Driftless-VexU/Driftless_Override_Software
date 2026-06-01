#include "driftless/robot/commands/holonomic_drive_train/SetMotionVectorCommand.hpp"

namespace driftless::robot::commands::holonomic_drive_train {
SetMotionVectorCommand::SetMotionVectorCommand(double x_velocity,
                                               double y_velocity,
                                               double angular_velocity,
                                               bool is_normal)
    : m_x_velocity(x_velocity),
      m_y_velocity(y_velocity),
      m_angular_velocity(angular_velocity),
      m_is_normal(is_normal) {}

bool SetMotionVectorCommand::isNormal() const { return m_is_normal; }

double SetMotionVectorCommand::getXVelocity() const { return m_x_velocity; }

double SetMotionVectorCommand::getYVelocity() const { return m_y_velocity; }

double SetMotionVectorCommand::getAngularVelocity() const {
  return m_angular_velocity;
}
}  // namespace driftless::robot::commands::holonomic_drive_train
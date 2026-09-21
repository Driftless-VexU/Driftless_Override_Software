#include "driftless/robot/subsystems/odometry/OdometrySubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
void OdometrySubsystem::handleCommand(
    const commands::odometry::SetHeadingCommand& cmd) {
  if (m_position_tracker) {
    m_position_tracker->setTheta(cmd.m_heading);
  } else {
    throw std::invalid_argument(
        "No valid position tracker is set for the odometry subsystem");
  }
}

void OdometrySubsystem::handleCommand(
    const commands::odometry::SetPositionCommand& cmd) {
  if (m_position_tracker) {
    m_position_tracker->setPosition(
        Position{cmd.m_x_pos, cmd.m_y_pos, cmd.m_theta_pos});
  } else {
    throw std::invalid_argument(
        "No valid position tracker is set for the odometry subsystem");
  }
}

void OdometrySubsystem::handleCommand(
    const commands::odometry::SetXPositionCommand& cmd) {
  if (m_position_tracker) {
    m_position_tracker->setX(cmd.m_x_position);
  } else {
    throw std::invalid_argument(
        "No valid position tracker is set for the odometry subsystem");
  }
}

void OdometrySubsystem::handleCommand(
    const commands::odometry::SetYPositionCommand& cmd) {
  if (m_position_tracker) {
    m_position_tracker->setY(cmd.m_y_position);
  } else {
    throw std::invalid_argument(
        "No valid position tracker is set for the odometry subsystem");
  }
}

void OdometrySubsystem::handleCommand(const auto& cmd) {
  throw std::invalid_argument(
      "No behavior defined to handle the command of type: " +
      std::string(typeid(cmd).name()) + " for the odometry subsystem");
}

OdometrySubsystem::OdometrySubsystem(
    std::unique_ptr<IPositionTracker>& position_tracker)
    : ASubsystem{ESubsystem::ODOMETRY},
      m_position_tracker{std::move(position_tracker)} {}

void OdometrySubsystem::init() {
  if (m_position_tracker) {
    m_position_tracker->init();
  }
}

void OdometrySubsystem::run() {
  if (m_position_tracker) {
    m_position_tracker->run();
  }
}

void OdometrySubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& arg) { this->handleCommand(arg); }, cmd);
  /*
  if (command_name == ESubsystemCommand::ODOMETRY_SET_POSITION) {
    if (m_position_tracker) {
      Position position{va_arg(args, double), va_arg(args, double),
                        va_arg(args, double)};
      m_position_tracker->setPosition(position);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_X) {
    if (m_position_tracker) {
      double x{va_arg(args, double)};
      m_position_tracker->setX(x);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_Y) {
    if (m_position_tracker) {
      double y{va_arg(args, double)};
      m_position_tracker->setY(y);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_THETA) {

  }
    */
}

void* OdometrySubsystem::state(ESubsystemState state_name) {
  void* result{};

  if (state_name == ESubsystemState::ODOMETRY_GET_POSITION) {
    if (m_position_tracker) {
      Position* position{new Position{m_position_tracker->getPosition()}};
      result = position;
    }
  }

  return result;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
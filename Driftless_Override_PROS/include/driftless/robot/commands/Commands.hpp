#ifndef __ROBOT_COMMANDS_HPP__
#define __ROBOT_COMMANDS_HPP__

#include <variant>

/* ADD COMMAND HEADERS HERE */
#include "driftless/robot/commands/holonomic_drive_train/SetAngularVelocityCommand.hpp"
#include "driftless/robot/commands/holonomic_drive_train/SetLinearVelocityCommand.hpp"
#include "driftless/robot/commands/holonomic_drive_train/SetMotionVectorCommand.hpp"
#include "driftless/robot/commands/odometry/SetHeadingCommand.hpp"
#include "driftless/robot/commands/odometry/SetPositionCommand.hpp"
#include "driftless/robot/commands/odometry/SetXPositionCommand.hpp"
#include "driftless/robot/commands/odometry/SetYPositionCommand.hpp"
#include "driftless/robot/commands/tank_drive_train/SetVelocityCommand.hpp"
#include "driftless/robot/commands/tank_drive_train/SetVoltageCommand.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

using Command =
    std::variant<holonomic_drive_train::SetMotionVectorCommand,
                 holonomic_drive_train::SetLinearVelocityCommand,
                 holonomic_drive_train::SetAngularVelocityCommand,
                 odometry::SetPositionCommand, odometry::SetXPositionCommand,
                 odometry::SetYPositionCommand, odometry::SetHeadingCommand,
                 tank_drive_train::SetVelocityCommand,
                 tank_drive_train::SetVoltageCommand>;

}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
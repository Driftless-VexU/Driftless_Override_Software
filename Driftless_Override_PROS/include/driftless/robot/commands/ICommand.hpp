#ifndef __I_COMMAND_HPP__
#define __I_COMMAND_HPP__

#include "driftless/robot/subsystems/ESubsystem.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief Generic interface for a command which can be sent to the robot
/// @author Matthew Backman
class ICommand {
 public:
  ~ICommand() = default;

  /// @brief Retrieves the target subsystem for this command
  /// @return __subsystems::ESubsystem__ The name of the target subsystem
  virtual subsystems::ESubsystem getTarget() const = 0;
};
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
#ifndef __DISABLE_COMMAND_HPP__
#define __DISABLE_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which don't belong to any subsystem
/// @author Jacob Fiedorowicz
namespace misc {

/// @brief Command to lower the current in a subsystem
/// @author Jacob Fiedorowicz
struct DisableCommand {};
}  // namespace misc
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
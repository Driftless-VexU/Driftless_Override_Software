#ifndef __TOGGLE_INTAKE_PISTON_HEIGHT_COMMAND_HPP__
#define __TOGGLE_INTAKE_PISTON_HEIGHT_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the brakes subsystem
/// @author Jacob Fiedorowicz
namespace intake {

/// @brief Command to toggle the piston controlling the height of the intake
/// @author Jacob Fiedorowicz
struct ToggleIntakePistonHeightCommand {};
}  // namespace intake
}  // namespace commands
}  // namespace robot
}  // namespace driftless
#endif

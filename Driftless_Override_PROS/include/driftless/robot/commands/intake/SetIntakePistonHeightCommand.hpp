#ifndef __SET_INTAKE_PISTON_HEIGHT_COMMAND_HPP__
#define __SET_INTAKE_PISTON_HEIGHT_COMMAND_HPP__

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

/// @brief Command to either deploy or retract the piston controlling the intake height
/// @author Jacob Fiedorowicz
struct SetIntakePistonHeightCommand {
    /// @brief Wether the piston is deployed or not
    bool m_deployed{};
};
}  // namespace brakes
}  // namespace commands
}  // namespace robot
}  // namespace driftless
#endif

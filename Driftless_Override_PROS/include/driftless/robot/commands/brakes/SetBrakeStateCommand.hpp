#ifndef __SET_BRAKE_STATE_COMMAND_HPP__
#define __SET_BRAKE_STATE_COMMAND_HPP__

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
/// @author Matthew Backman
namespace brakes {

/// @brief Command to set the state of the brakes
/// @author Matthew Backman
struct SetBrakeStateCommand {
  bool m_deployed{};
};
}  // namespace brakes
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
#ifndef __CLAW_SET_ELBOW_ROTATION_COMMAND_HPP__
#define __CLAW_SET_ELBOW_ROTATION_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief Namespace for commands to be sent to the claw subsystem
/// @author Matthew Backman
namespace claw {

/// @brief Command to set the rotation of the elbow joint, between [0, PI]
/// @author Matthew Backman
struct ClawSetElbowRotationCommand {
  float m_rotation;
};
}  // namespace claw
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
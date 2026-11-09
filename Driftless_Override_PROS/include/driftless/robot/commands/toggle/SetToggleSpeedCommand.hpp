#ifndef __SET_TOGGLE_SPEED_COMMAND_HPP__
#define __SET_TOGGLE_SPEED_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the toggle subsystem
/// @author Jacob Fiedorowicz
namespace toggle {

/// @brief Command to set the speed of the toggle
/// @author Jacob Fiedorowicz
struct SetToggleSpeedCommand {
    float m_motor_speed; 
};
}
}
}
}

#endif
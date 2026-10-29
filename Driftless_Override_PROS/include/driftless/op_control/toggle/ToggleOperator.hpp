#ifndef __TOGGLE_OPERATOR_HPP__
#define __TOGGLE_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for toggle control during operator control
/// @author Jacob Fiedorowicz
namespace toggle {

/// @brief Operator class for the toggle subsystem
/// @author Jacob Fiedorowicz
class ToggleOperator {
public:
/// @brief Construct a new Toggle Operator object
/// @param profile __const std::shared_ptr<io::Icontroller>&__ The controller to recieve from 
/// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
ToggleOperator(const std::shared_ptr<io::IController>& controller,
const std::shared_ptr<robot::Robot>& robot);

/// @brief Updates the toggle using the configuration within the given profile
/// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile to use for controls
void update(const std::unique_ptr<profiles::IProfile>& profile);

private:
std::shared_ptr<io::IController> m_controller{};

std::shared_ptr<robot::Robot> m_robot{};

/// @brief Sets the motor speed for the toggle
void setMotorSpeed(float speed);
};
}
}
}

#endif
#ifndef __INTAKE_OPERATOR_HPP__
#define __INTAKE_OPERATOR_HPP__

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

/// @brief Namespace for intake control during operator control
/// @author Jacob Fiedorowicz
namespace intake {

/// @brief Operator class for the intake subsystem
/// @author Jacob Fiedorowicz
class IntakeOperator {
 public:
  /// @brief Construct a new Intake Operator Object
  /// @param controller __const std::shared_ptr<io::IController>&__ The
  /// controller to recieve input from
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
  IntakeOperator(const std::shared_ptr<io::IController>& controller,
                 const std::shared_ptr<robot::Robot>& robot);
    /// @brief Updates the intake using the configuration within the given profile
    /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile to use for the controls
    void update(const std::unique_ptr<profiles::IProfile>& profile);

    private:
    std::shared_ptr<io::IController> m_controller{};
    std::shared_ptr<robot::Robot> m_robot{};

    /// @brief Toggles the state of the intake piston
    void toggleIntakePistonUpDown();

    /// @brief Sets the voltage going to the intake motor
    /// @param voltage __float__ The volts to send to the motor
    void setIntakeMotorVoltage(float voltage);

    /// @brief Sets the state of the intake piston
    /// @param state __bool__ The state to set the piston. True for down, false for up
    void setIntakePistonUpDown(bool state);
};
}  // namespace intake
}  // namespace op_control
}  // namespace driftless
#endif
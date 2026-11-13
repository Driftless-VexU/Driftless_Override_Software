#ifndef __CLAW_OPERATOR_HPP__
#define __CLAW_OPERATOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
#include "driftless/io/IController.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for controlling the claw during op control
/// @author Matthew Backman
namespace claw {

/// @brief Class to manage control of the claw during op control
/// @author Matthew Backman
class ClawOperator {
 public:
  /// @brief Construct a new Claw Operator object
  /// @param controller __std::shared_ptr<io::IController>__ The controller to
  /// recieve input from
  /// @param robot __std::shared_ptr<robot::Robot>__ The robot to control
  ClawOperator(std::shared_ptr<io::IController> controller,
               std::shared_ptr<robot::Robot> robot);

  /// @brief Updates the claw in operator control using the provided keybindings
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile
  /// to use for keybinds
  void update(const std::unique_ptr<profiles::IProfile>& profile);

 private:
  std::shared_ptr<robot::Robot> m_robot;

  std::shared_ptr<io::IController> m_controller;

  /// @brief Wrapper function to send the toggle claw pivot orientation command
  /// to the robot
  inline void toggleClawPivot();

  /// @brief Wrapper function to send the toggle claw open closed command to the
  /// robot
  inline void toggleClawOpenClose();
};
}  // namespace claw
}  // namespace op_control
}  // namespace driftless

#endif
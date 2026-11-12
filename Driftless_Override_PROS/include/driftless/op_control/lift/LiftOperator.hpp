#ifndef __LIFT_OPERATOR_HPP__
#define __LIFT_OPERATOR_HPP__

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

/// @brief Namespace for brakes control during operator control
/// @author Jacob Fiedorowicz
namespace lift {

/// @brief Operator class for the lift subsystem
/// @author Jacob Fiedorowicz
class LiftOperator {
 public:
  /// @brief Construct a new Lift Operator object
  /// @param controller __const std::shared_ptr<io::IController>&__ The
  /// controller to recieve input from
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
  LiftOperator(const std::shared_ptr<io::IController>& controller,
               const std::shared_ptr<robot::Robot>& robot);

  /// @brief Updates the lift using the configuration within the given profile
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile
  /// to use for controls
  void update(const std::unique_ptr<profiles::IProfile>& profile);

  /// @brief Gets the height the lift needs to be set to after the button is let go
  /// @param height __float__ The height to set the lift to
  float getLiftHeight();

  /// @brief Raises the lift while button is held
  void setLiftHeight(float height);

 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  float target_height{};
};
}  // namespace lift
}  // namespace op_control
}  // namespace driftless
#endif
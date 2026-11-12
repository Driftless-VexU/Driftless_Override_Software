#ifndef __WALL_OPERATOR_HPP__
#define __WALL_OPERATOR_HPP__

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

/// @brief Namespace for wall control during operator control
/// @author Jacob Fiedorowicz
namespace wall {

/// @brief Operator class for the wall subsystem
/// @author Jacob Fiedorowicz
class WallOperator {
 public:
  /// @brief Constructs a new Wall Operator object
  /// @param controller __const ::std::shared_ptr<io::IController>&__ the
  /// controller to recieve input from
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
  WallOperator(const std::shared_ptr<io::IController>& controller,
               const std::shared_ptr<robot::Robot>& robot);

  /// @brief Updates the wall using the configuration within the given profile
  /// @param profile __const std::shared_ptr<profiles::IProfile>&__ The profile
  /// to use for controls
  void update(const std::unique_ptr<profiles::IProfile>& profile);

 private:
  std::shared_ptr<io::IController> m_controller;

  std::shared_ptr<robot::Robot> m_robot;

  inline void toggleWall();

  inline void setWallWheelState(bool deployed);

  inline void setWallMotorVoltage(float voltage);
};
}  // namespace wall
}  // namespace op_control
}  // namespace driftless

#endif
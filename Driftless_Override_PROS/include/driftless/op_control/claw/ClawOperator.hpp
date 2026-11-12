#ifndef __CLAW_OPERATOR_HPP__
#define __CLAW_OPERATOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
#include "driftless/io/IController.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/profiles/IProfile.hpp"

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
 ClawOperator(std::shared_ptr<robot::Robot> robot, std::shared_ptr<io::IController> controller);

 void update(const std::unique_ptr<profiles::IProfile>& profile);
 private:
  std::shared_ptr<robot::Robot> m_robot;

  std::shared_ptr<io::IController> m_controller;

  inline void toggleClawPivot();

  inline void toggleClawOpenClose();
};
}  // namespace claw
}  // namespace op_control
}  // namespace driftless

#endif
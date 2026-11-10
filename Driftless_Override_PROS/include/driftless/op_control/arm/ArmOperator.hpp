#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"


/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for arm operation during operator control
/// @author Matthew Backman
namespace arm {

/// @brief Class controlling the arm subsystem during operator control
/// @author Matthew Backman
class ArmOperator {
 public:
  ArmOperator(const std::shared_ptr<io::IController>& controller,
              const std::shared_ptr<robot::Robot>& robot);

  void update(const std::unique_ptr<profiles::IProfile>& profile);

 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  inline void goLoad();

  inline void goScoreLow();

  inline void goScoreHigh();

  inline void resetArm();

  inline float getLiftHeight();
};
}  // namespace arm
}  // namespace op_control
}  // namespace driftless

#endif
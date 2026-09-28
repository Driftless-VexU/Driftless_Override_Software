#ifndef __CONTINUOUS_LIFT_BUILDER_HPP__
#define __CONTINUOUS_LIFT_BUILDER_HPP__

#include "driftless/robot/subsystems/lift/ContinuousLift.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the lift subsystem code
/// @author Matthew Backman
namespace lift {

class ContinuousLiftBuilder {
  friend class ContinuousLift;

 public:
  ContinuousLiftBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

  [[nodiscard]]
  ContinuousLift build() &&;

  [[nodiscard]]
  std::unique_ptr<ContinuousLift> buildUnique() &&;

 private:
  hal::MotorGroup m_motors{};
};
}  // namespace lift
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
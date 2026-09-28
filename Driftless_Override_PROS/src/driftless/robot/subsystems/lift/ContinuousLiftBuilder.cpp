#include "driftless/robot/subsystems/lift/ContinuousLiftBuilder.hpp"

namespace driftless::robot::subsystems::lift {
ContinuousLiftBuilder&& ContinuousLiftBuilder::withMotor(
    std::unique_ptr<io::IMotor> motor) && {
  m_motors.addMotor(motor);
  return std::move(*this);
}

ContinuousLift ContinuousLiftBuilder::build() && {
  return ContinuousLift{std::move(*this)};
}

std::unique_ptr<ContinuousLift> ContinuousLiftBuilder::buildUnique() && {
  return std::unique_ptr<ContinuousLift>{new ContinuousLift{std::move(*this)}};
}

}  // namespace driftless::robot::subsystems::lift
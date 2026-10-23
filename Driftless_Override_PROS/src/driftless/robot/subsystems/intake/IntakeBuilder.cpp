#include "driftless/robot/subsystems/intake/IntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
IntakeBuilder&& IntakeBuilder::withPiston(
    std::unique_ptr<io::IPiston> piston) && {
  m_piston = std::move(piston);
  return std::move(*this);
}
IntakeBuilder&& IntakeBuilder::withMotor(std::unique_ptr<io::IMotor> motor) && {
  m_motor.addMotor(std::move(motor));
  return std::move(*this);
}
Intake IntakeBuilder::build() && { return Intake{std::move(*this)}; }
std::unique_ptr<Intake> IntakeBuilder::buildUnique() && {
  return std::unique_ptr<Intake>{new Intake{std::move(*this)}};
}
}  // namespace driftless::robot::subsystems::intake
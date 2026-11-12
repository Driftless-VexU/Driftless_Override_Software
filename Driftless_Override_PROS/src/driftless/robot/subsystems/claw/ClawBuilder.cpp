#include "driftless/robot/subsystems/claw/ClawBuilder.hpp"

namespace driftless::robot::subsystems::claw {
ClawBuilder&& ClawBuilder::withPiston(std::unique_ptr<io::IPiston> piston) && {
  m_piston = std::move(piston);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withMotor(std::unique_ptr<io::IMotor> motor) && {
  m_pivot_motor.addMotor(std::move(motor));
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withTask(std::unique_ptr<rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withMutex(std::unique_ptr<rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withDelayer(std::unique_ptr<rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

ClawBuilder&& ClawBuilder::withMotorRotationsToPivotRadians(
    float motorRotationsToPivotRadians) && {
  m_motor_rotations_to_pivot_radians = motorRotationsToPivotRadians;
  return std::move(*this);
}

std::unique_ptr<Claw> ClawBuilder::buildUnique() && {
  return std::unique_ptr<Claw>{new Claw(std::move(*this))};
}
Claw ClawBuilder::build() && { return Claw(std::move(*this)); }

}  // namespace driftless::robot::subsystems::claw
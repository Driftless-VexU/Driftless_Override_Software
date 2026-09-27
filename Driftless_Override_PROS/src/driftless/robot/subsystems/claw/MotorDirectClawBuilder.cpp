#include "driftless/robot/subsystems/claw/MotorDirectClawBuilder.hpp"

#include "driftless/robot/subsystems/claw/MotorDirectClaw.hpp"

namespace driftless::robot::subsystems::claw {
MotorDirectClawBuilder&& MotorDirectClawBuilder::withElbowMotor(
    std::unique_ptr<io::IMotor> motor) && {
  m_elbow_motors.addMotor(motor);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withWristMotor(
    std::unique_ptr<io::IMotor> motor) && {
  m_wrist_motors.addMotor(motor);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withFlipMotor(
    std::unique_ptr<io::IMotor> motor) && {
  m_flip_motors.addMotor(motor);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withElbowPID(
    control::PID pid) && {
  m_elbow_pid = pid;
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withWristPID(
    control::PID pid) && {
  m_wrist_pid = pid;
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withFlipPID(
    control::PID pid) && {
  m_flip_pid = pid;
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withTask(
    std::unique_ptr<rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withMutex(
    std::unique_ptr<rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withMotorToElbowRotations(
    float motor_to_elbow_rotations) && {
  m_motor_to_elbow_rotations = motor_to_elbow_rotations;
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withMotorToWristRotations(
    float motor_to_wrist_rotations) && {
  m_motor_to_wrist_rotations = motor_to_wrist_rotations;
  return std::move(*this);
}

MotorDirectClawBuilder&& MotorDirectClawBuilder::withMotorToFlipRotations(
    float motor_to_flip_rotations) && {
  m_motor_to_flip_rotations = motor_to_flip_rotations;
  return std::move(*this);
}

MotorDirectClaw MotorDirectClawBuilder::build() && {
  return MotorDirectClaw{std::move(*this)};
}

std::unique_ptr<MotorDirectClaw> MotorDirectClawBuilder::buildUnique() && {
  return std::unique_ptr<MotorDirectClaw>{
      new MotorDirectClaw{std::move(*this)}};
}
}  // namespace driftless::robot::subsystems::claw
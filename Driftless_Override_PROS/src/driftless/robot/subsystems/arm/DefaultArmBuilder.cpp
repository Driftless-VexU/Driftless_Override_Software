#include "driftless/robot/subsystems/arm/DefaultArmBuilder.hpp"

#include "driftless/robot/subsystems/arm/DefaultArm.hpp"


namespace driftless::robot::subsystems::arm {
DefaultArmBuilder&& DefaultArmBuilder::withElbowMotor(
    std::unique_ptr<io::IMotor> elbow_motor) && {
  m_elbow_motors.addMotor(std::move(elbow_motor));
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withWristMotor(
    std::unique_ptr<io::IMotor> wrist_motor) && {
  m_wrist_motors.addMotor(std::move(wrist_motor));
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withElbowPID(control::PID elbow_pid) && {
  m_elbow_pid = elbow_pid;
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withWristPID(control::PID wrist_pid) && {
  m_wrist_pid = wrist_pid;
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withTask(
    std::unique_ptr<rtos::ITask> task) && {
  m_task = std::move(task);
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withMutex(
    std::unique_ptr<rtos::IMutex> mutex) && {
  m_mutex = std::move(mutex);
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer> delayer) && {
  m_delayer = std::move(delayer);
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withRobot(
    std::weak_ptr<Robot> robot) && {
  m_robot = robot;
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withArmTargets(
    DefaultArmTargetPositions arm_targets) && {
  m_arm_targets = arm_targets;
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withElbowPositionTolerance(
    float elbow_position_tolerance) && {
  m_elbow_position_tolerance = elbow_position_tolerance;
  return std::move(*this);
}

DefaultArmBuilder&& DefaultArmBuilder::withWristPositionTolerance(
    float wrist_position_tolerance) && {
  m_wrist_position_tolerance = wrist_position_tolerance;
  return std::move(*this);
}

DefaultArm DefaultArmBuilder::build() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for "
        "DefaultArm");
  }

  return DefaultArm{std::move(*this)};
}

std::unique_ptr<DefaultArm> DefaultArmBuilder::buildUnique() && {
  if (!m_delayer || !m_mutex || !m_task) {
    throw std::runtime_error(
        "One or more RTOS components not set for "
        "SparkFunPositionTrackerBuilder");
  }

  return std::unique_ptr<DefaultArm>{new DefaultArm{std::move(*this)}};
}
}  // namespace driftless::robot::subsystems::arm
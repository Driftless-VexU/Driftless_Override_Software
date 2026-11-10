#ifndef __DEFAULT_ARM_BUILDER_HPP__
#define __DEFAULT_ARM_BUILDER_HPP__

#include "driftless/robot/subsystems/arm/DefaultArm.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the arm subsystem
/// @author Matthew Backman
namespace arm {

class DefaultArmBuilder {
 public:
  DefaultArmBuilder&& withElbowMotor(
      std::unique_ptr<io::IMotor> elbow_motor) &&;

  DefaultArmBuilder&& withWristMotor(
      std::unique_ptr<io::IMotor> wrist_motor) &&;

  DefaultArmBuilder&& withElbowPID(control::PID elbow_pid) &&;

  DefaultArmBuilder&& withWristPID(control::PID wrist_pid) &&;

  DefaultArmBuilder&& withTask(std::unique_ptr<rtos::ITask> task) &&;

  DefaultArmBuilder&& withMutex(std::unique_ptr<rtos::IMutex> mutex) &&;

  DefaultArmBuilder&& withDelayer(std::unique_ptr<rtos::IDelayer> delayer) &&;

  DefaultArmBuilder&& withRobot(std::weak_ptr<Robot> robot) &&;

  DefaultArmBuilder&& withArmTargets(DefaultArmTargetPositions arm_targets) &&;

  DefaultArmBuilder&& withElbowPositionTolerance(
      float elbow_position_tolerance) &&;

  DefaultArmBuilder&& withWristPositionTolerance(
      float wrist_position_tolerance) &&;

  [[nodiscard]]
  DefaultArm build() &&;

  [[nodiscard]]
  std::unique_ptr<DefaultArm> buildUnique() &&;

 private:
  hal::MotorGroup m_elbow_motors{};

  hal::MotorGroup m_wrist_motors{};

  control::PID m_elbow_pid{};

  control::PID m_wrist_pid{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::weak_ptr<Robot> m_robot{};

  DefaultArmTargetPositions m_arm_targets{};

  // POSITION TOLERANCES

  float m_elbow_position_tolerance{0.0f};

  float m_wrist_position_tolerance{0.0f};
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
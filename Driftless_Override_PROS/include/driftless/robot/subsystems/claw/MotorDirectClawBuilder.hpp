#ifndef __MOTOR_DIRECT_CLAW_BUILDER_HPP__
#define __MOTOR_DIRECT_CLAW_BUILDER_HPP__

#include "driftless/robot/subsystems/claw/MotorDirectClaw.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace containing the claw subsystem
/// @author Matthew Backman
namespace claw {

/// @brief Builder for MotorDirectClaw objects
/// @author Matthew Backman
class MotorDirectClawBuilder {
  friend class MotorDirectClaw;

 public:
  /// @brief Add a motor to control the elbow joint of the claw
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor to add
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withElbowMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Add a motor to control the wrist joint of the claw
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor to add
  /// @return  __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withWristMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Add a motor to control the flip orientation of the claw
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor to add
  /// @return  __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withFlipMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Add a PID controller to manage the elbow motor's motion
  /// @param pid __control::PID__ The pid controller to use
  /// @return  __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withElbowPID(control::PID pid) &&;

  /// @brief Add a PID controller to manage the wrist motor's motion
  /// @param pid __control::PID__ The pid controller to use
  /// @return  __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withWristPID(control::PID pid) &&;

  /// @brief Add a PID controller to manage the flip motor's motion
  /// @param pid __control::PID__ The pid controller to use
  /// @return  __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withFlipPID(control::PID pid) &&;

  /// @brief Add a task object to run background processes
  /// @param task __std::unique_ptr<rtos::ITask>__ The task to add
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withTask(std::unique_ptr<rtos::ITask> task) &&;

  /// @brief Add a mutex to support multitasking
  /// @param mutex __std::unique_ptr<rtos::IMutex>__ The mutex to add
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withMutex(std::unique_ptr<rtos::IMutex> mutex) &&;

  /// @brief Add a delayer to support multitasking
  /// @param delayer __std::unique_ptr<rtos::IDelayer>__ The delayer to add
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withDelayer(
      std::unique_ptr<rtos::IDelayer> delayer) &&;

  /// @brief Add a conversion rate between motor rotations and elbow joint
  /// rotations
  /// @param motor_to_elbow_rotations __float__ The conversion rate between
  /// motor rotations and elbow joint rotations
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withMotorToElbowRotations(
      const float motor_to_elbow_rotations) &&;

  /// @brief Add a conversion rate between motor rotations and wrist joint
  /// rotations
  /// @param motor_to_wrist_rotations __float__ The conversion rate between
  /// motor rotations and wrist joint rotations
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withMotorToWristRotations(
      const float motor_to_wrist_rotations) &&;

  /// @brief Add a conversion rate between motor rotations and flip joint
  /// rotations
  /// @param motor_to_flip_rotations __float__ The conversion rate between motor
  /// rotations and flip joint rotations
  /// @return __MotorDirectClawBuilder&&__ The current builder
  MotorDirectClawBuilder&& withMotorToFlipRotations(
      const float motor_to_flip_rotations) &&;

  /// @brief Consume the builder to create a new MotorDirectClaw object
  /// @return __MotorDirectClaw__ The new MotorDirectClaw object
  [[nodiscard]]
  MotorDirectClaw build() &&;

  /// @brief Consume the builder to create a new MotorDirectClaw object wrapped
  /// in a unique_ptr
  /// @return __std::unique_ptr<MotorDirectClaw>__ A unique_ptr to the new
  /// MotorDirectClaw object
  [[nodiscard]]
  std::unique_ptr<MotorDirectClaw> buildUnique() &&;

 private:
  hal::MotorGroup m_elbow_motors{};

  hal::MotorGroup m_wrist_motors{};

  hal::MotorGroup m_flip_motors{};

  control::PID m_elbow_pid{};

  control::PID m_wrist_pid{};

  control::PID m_flip_pid{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  float m_motor_to_elbow_rotations{};

  float m_motor_to_wrist_rotations{};

  float m_motor_to_flip_rotations{};
};

}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
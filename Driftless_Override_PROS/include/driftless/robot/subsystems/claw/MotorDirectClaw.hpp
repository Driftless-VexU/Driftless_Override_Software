#ifndef __MOTOR_DIRECT_DRIVE_HPP__
#define __MOTOR_DIRECT_DRIVE_HPP__

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/claw/IClaw.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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

class MotorDirectClawBuilder;

/// @brief Claw driver using separate motors for each joint.
/// @author Matthew Backman
class MotorDirectClaw : public IClaw {
  friend class MotorDirectClawBuilder;

 public:
  /// @brief Initialize the Motor Direct Claw object
  void init() override;

  /// @brief Run the Motor Direct Claw object
  void run() override;

  /// @brief Set the rotation of the elbow joint of the claw
  /// @param target __float__ The target rotation of the elbow joint
  void setElbowRotation(float target) override;

  /// @brief Set the rotation of the wrist joint of the claw
  /// @param target __float The target rotation of the wrist joint
  void setWristRotation(float target) override;

  /// @brief Flips the claw upside down
  void flipClaw() override;

  /// @brief Get the rotation of the elbow joint of the claw
  /// @return  __float__ The rotation of the elbow joint
  float getElbowRotation() override;

  /// @brief Get the rotation of the wrist joint of the claw
  /// @return  __float__ The rotation of the wrist joint
  float getWristRotation() override;

  /// @brief Determines if the claw is flipped
  /// @return __bool__ Returns true if the claw is flipped, false if the claw is
  /// not flipped
  bool isClawFlipped() override;

 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Continuously loop the task update of the provided MotorDirectClaw
  /// object
  /// @param params __void*__ The claw to update
  static void taskLoop(void* params);

  hal::MotorGroup m_elbow_motors{};

  hal::MotorGroup m_wrist_motors{};

  hal::MotorGroup m_flip_motors{};

  control::PID m_elbow_pid{};

  control::PID m_wrist_pid{};

  control::PID m_flip_pid{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  const float m_motor_to_elbow_rotations{};

  const float m_motor_to_wrist_rotations{};

  const float m_motor_to_flip_rotations{};

  bool m_is_flipped{};

  /// @brief Construct a new Motor Direct Claw object
  /// @param builder __MotorDirectClawBuilder&&__ The builder consumed to create
  /// this object
  explicit MotorDirectClaw(MotorDirectClawBuilder&& builder);

  /// @brief Updates the state of the claw
  void taskUpdate();
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
#ifndef __DEFAULT_ARM_HPP__
#define __DEFAULT_ARM_HPP__

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/arm/IArm.hpp"
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

/// @brief Namespace for the arm subsystem
/// @author Matthew Backman
namespace arm {

class DefaultArmBuilder;

/// @brief Default implementation of the arm subsystem
/// @author Matthew Backman
class DefaultArm : public IArm {
  friend class DefaultArmBuilder;

 public:
  /// @brief Initialize all devices in the arm
  void init() override;

  /// @brief Run the arm and start background tasks, if any
  void run() override;

  /// @brief Move the wrist to a hard-stop and reset position
  void resetWrist() override;

  /// @brief Move the arm to the load position
  void goToLoadPosition() override;

  /// @brief Move the arm to the scan position
  void goToScanPosition() override;

  /// @brief Move the arm to the lower scoring position
  void goToScoreLowPosition() override;

  /// @brief Move the arm to the higher scoring position
  void goToScoreHighPosition() override;

  /// @brief Get the current position state of the arm
  /// @return __EArmState__ The current position state of the arm
  EArmState getCurrentPositionState() override;

 private:
  static constexpr uint8_t TASK_DELAY{20};

  static constexpr uint16_t TASK_TIMEOUT{2000};

  /// @brief Run arm updates constantly in a background task
  /// @param params __void*__ Pointer to the arm instance
  static void taskLoop(void* params);

  hal::MotorGroup m_elbow_motors{};

  hal::MotorGroup m_wrist_motors{};

  control::PID m_elbow_pid{};

  control::PID m_wrist_pid{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  Robot& m_robot;

  // LIFT POSITIONS

  const float m_lift_load_height{0.0f};

  const float m_lift_load_transition_height{0.0f};

  // ELBOW POSITIONS

  const float m_elbow_load_position{0.0f};

  const float m_elbow_score_low_position{0.0f};

  const float m_elbow_score_high_position{0.0f};

  // WRIST POSITIONS

  const float m_wrist_load_position{0.0f};

  const float m_wrist_scan_position{0.0f};

  const float m_wrist_score_low_position{0.0f};

  const float m_wrist_score_high_position{0.0f};

  // ACTIVELY TARGETED POSITIONS

  float m_elbow_target_position{0.0f};

  float m_wrist_target_position{0.0f};

  // POSITION TOLERANCES

  float m_elbow_position_tolerance{0.0f};

  float m_wrist_position_tolerance{0.0f};

  EArmState m_current_position_state{EArmState::START};

  EArmState m_target_position_state{EArmState::START};

  bool m_is_at_target{};

  bool m_is_initialized{};

  /// @brief Construct a new Default Arm object
  /// @param builder __DefaultArmBuilder__ The builder to use for constructing
  /// the arm
  DefaultArm(DefaultArmBuilder&& builder);

  /// @brief Update the arm based on the current state
  void taskUpdate();

  inline void handleLoadState();

  inline void handleScanState();

  inline void handleScoreLowState();

  inline void handleScoreHighState();

  /// @brief Get the height of the lift, in inches
  /// @return __float__ The height of the lift, in inches
  inline float getLiftHeight();

  /// @brief Set the height of the lift, in inches
  /// @param height __float__ The height of the lift, in inches
  inline void setLiftHeight(float height);

  /// @brief Determine if the claw is flipped
  /// @return __bool__ True if the claw is flipped, false if not
  inline bool isClawFlipped();

  /// @brief Flip the claw to the given orientation
  /// @param orientation __bool__ True to flip the claw, false for normal
  /// orientation
  inline void flipClaw(bool orientation);
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
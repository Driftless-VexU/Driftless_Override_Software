#ifndef __Continuous_Lift_HPP__
#define __Continuous_Lift_HPP__

#include <cstdint>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/lift/ILift.hpp"
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

/// @brief The namespace for the lift subsystem
/// @author Jacob Fiedorowicz
namespace lift {

class DirectContinuousLiftBuilder;

/// @brief Lift implementation using continuous control
class DirectContinuousLift : public ILift {
  friend class DirectContinuousLiftBuilder;

 public:
  /// @brief Initialize the ContinuousLift
  void init() override;

  /// @brief Run the ContinuousLift
  void run() override;

  /// @brief Set the height of the lift
  /// @param height __float__ The height of the lift
  void setHeight(float height) override;

  /// @brief Get the current height of the lift
  /// @return __float__ The current height of the lift
  float getHeight() override;

 private:
  static constexpr uint8_t TASK_DELAY{10};

  static void taskLoop(void* args);

  /// @brief The motors used to control the lift
  hal::MotorGroup m_motors{};

  /// @brief The PID controller for the lift
  control::PID m_pid{};

  /// @brief The task for thel ift
  std::unique_ptr<rtos::ITask> m_task{};

  /// @brief delays things
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  /// @brief Only allows one thing to run at a time
  std::unique_ptr<rtos::IMutex> m_mutex{};

  /// @brief Conversion factor for rotations to inches
  float m_rotations_to_inches{};

  /// @brief stores height
  float m_height{};

  /// @brief Updates the task
  void taskUpdate();

  /// @brief Constructs a DirectContinuousLift object
  /// @param builder __DirectContinuousLiftBuilder&&__ The builder creating this
  /// DirectContinousLift object. Consumed on use.
  DirectContinuousLift(DirectContinuousLiftBuilder&& Builder);
};
}  // namespace lift
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
#ifndef __MODULAR_HOLONOMIC_DRIVE_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_HPP__

#include <cmath>
#include <memory>
#include <vector>

#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/IHolonomicDriveModule.hpp"
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

/// @brief The namespace for the holonomic drive train subsystem code
/// @author Matthew Backman
namespace holonomic_drive_train {

class ModularHolonomicDriveBuilder;

/// @brief Class representing a modular holonomic drive train
/// @author Matthew Backman
class ModularHolonomicDrive : public IHolonomicDrive {
  friend class ModularHolonomicDriveBuilder;

 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly updates the modular holonomic drive
  /// @param params __void*__ The modular holonomic drive to update
  static void taskLoop(void* params);

  /// @brief The modules controlled by the drive train
  std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>
      m_modules{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  double m_max_linear_velocity{};

  double m_max_angular_velocity{};

  HolonomicMotionVector m_current_velocity{};

  ControlType m_control_type{ControlType::VOLTAGE};

  bool m_paused{false};

  /// @brief Constructs a ModularHolonomicDrive with the given builder
  /// @param builder __ModularHolonomicDriveBuilder&&__ The builder to construct
  /// the drive with
  explicit ModularHolonomicDrive(ModularHolonomicDriveBuilder&& builder);

  void taskUpdate();

 public:
 ModularHolonomicDrive(const ModularHolonomicDrive&) = delete;

 /// @brief Constructs a ModularHolonomicDrive by moving the given ModularHolonomicDrive
 /// @param  __ModularHolonomicDrive&&__ The ModularHolonomicDrive to move
 ModularHolonomicDrive(ModularHolonomicDrive&&) = default;

  /// @brief Initializes the modular holonomic drive
  void init() override;

  /// @brief Runs the modular holonomic drive
  void run() override;

  /// @brief Sets the motion vector of the modular holonomic drive
  /// @param motion_vector __HolonomicMotionVector__ Vector representing the
  /// desired motion of the robot (robot-centric)
  void setMotionVector(HolonomicMotionVector motion_vector) override;

  /// @brief Sets the linear velocity of the modular holonomic drive
  /// @param x_velocity The desired x velocity
  /// @param y_velocity The desired y velocity
  void setLinearVelocity(double x_velocity, double y_velocity) override;

  /// @brief Sets the angular velocity of the modular holonomic drive
  /// @param angular_velocity The desired angular velocity
  void setAngularVelocity(double angular_velocity) override;

  /// @brief Sets the normalized motion vector of the modular holonomic drive
  /// @param motion_vector Vector representing the desired normalized motion of
  /// the robot (robot-centric)
  void setNormalizedMotionVector(HolonomicMotionVector motion_vector) override;

  /// @brief Sets the normalized X velocity of the modular holonomic drive
  /// @param x_velocity The desired normalized X velocity [-1, 1]
  /// @param y_velocity The desired normalized Y velocity [-1, 1]
  void setNormalizedLinearVelocity(double x_velocity,
                                   double y_velocity) override;

  /// @brief Sets the normalized angular velocity of the modular holonomic drive
  /// @param angular_velocity The desired normalized angular velocity [-1, 1]
  void setNormalizedAngularVelocity(double angular_velocity) override;

  /// @brief Sets the voltage of a specific wheel in the holonomic drive train
  /// Only use for tuning or testing, not for regular control of the drive train
  /// @param wheel __int__ The index of the wheel to set the voltage of
  /// @param voltage __double__ The voltage to set the wheel to
  void setWheelVoltage(int wheel, double voltage) override;

  void setControlType(ControlType control_type) override;

  /// @brief Gets the speed of a specific wheel in the holonomic drive train
  /// @param wheel __int__ The index of the wheel to get the speed of
  /// @return __double__ The speed of the specified wheel in rad/s
  double getWheelSpeed(int wheel) override;
};
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif
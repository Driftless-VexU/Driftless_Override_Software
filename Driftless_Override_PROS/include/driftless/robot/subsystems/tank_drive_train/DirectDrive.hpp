#ifndef __DIRECT_DRIVE_HPP__
#define __DIRECT_DRIVE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/tank_drive_train/ITankDriveTrain.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the drivetrain subsystem code
/// @author Matthew Backman
namespace tank_drive_train {

class DirectDriveBuilder;

/// @brief Class representing the direct drive system
/// @author Matthew Backman
class DirectDrive : public ITankDriveTrain {
  friend class DirectDriveBuilder;

 private:
  hal::MotorGroup m_left_motors{};

  hal::MotorGroup m_right_motors{};

  double m_velocity_to_voltage{1.0};

  double m_gear_ratio{};

  double m_wheel_radius{};

  double m_drive_radius{};

  /// @brief Constructs a direct drive object from a builder
  /// @param builder __DirectDriveBuilder&&__ The builder being used to
  /// construct the drive
  explicit DirectDrive(DirectDriveBuilder&& builder);

 public:
  /// @brief Deleted copy constructor
  /// @param other __DirectDrive&__ The direct drive being copied
  DirectDrive(const DirectDrive& other) = delete;

  /// @brief Move constructor
  /// @param other __DirectDrive&&__ The direct drive being moved
  DirectDrive(DirectDrive&& other) = default;

  /// @brief Initializes the direct drive
  void init() override;

  /// @brief Runs the direct drive
  void run() override;

  /// @brief Sets the velocity of the drive train
  /// @param velocity __Velocity__ The desired velocity
  void setVelocity(Velocity velocity) override;

  /// @brief Sets the voltage sent to the drive train motors
  /// @param left_voltage __double__ The voltage passed to the left motors
  /// @param right_voltage __double__ The voltage passed to the right motors
  void setVoltage(double left_voltage, double right_voltage) override;

  /// @brief Gets the velocity of the drive train
  /// @return __Velocity__ The velocity of the drive train
  Velocity getVelocity() override;

  /// @brief Gets the radius of the drive train
  /// @return __double__ The radius of the drive train
  double getDriveRadius() const override;
};
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif
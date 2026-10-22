#ifndef __Intake_HPP__
#define __Intake_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the intake subsystem
/// @author Jacob Fiedorowicz
namespace intake {

// builder declaration, see IntakeBuilder.hpp for definition
class IntakeBuilder;

/// @brief Intake implementation
/// @author Jacob Fiedorowicz
class Intake : public IIntake {
  friend class IntakeBuilder;

 public:
  /// @brief Initialize the Intake object
  virtual void init() override;

  /// @brief Run the Intake object
  virtual void run() override;

  /// @brief Sets the state of the intake piston
  /// @param deployed __bool__ Whether the intake piston is deployed or not
  virtual void setPistonState(bool deployed) override;

  /// @brief Toggles the state of the intake piston
  virtual void togglePistonState() override;

  /// @brief Determine if the intake piston is deployed
  /// @return __bool__ Whether the intake piston is deployed or not
  virtual bool isPistonDeployed() override;

  /// @brief Sets the speed of the motor
  /// @param speed __float__ The speed of the motor
  virtual void setMotorSpeed(float speed) override;

 private:
  std::unique_ptr<io::IPiston> m_piston{};
  hal::MotorGroup m_motor{};

  /// @brief Construct a new Intake object
  /// @param builder __IntakeBuilder&&__ The builder creating this Intake
  /// object. Consumed on use.
  Intake(IntakeBuilder&& builder);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
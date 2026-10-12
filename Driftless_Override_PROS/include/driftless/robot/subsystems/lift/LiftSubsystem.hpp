#ifndef __LIFT_SUBSYSTEM_HPP__
#define __LIFT_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/lift/ILift.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the brakes subsystem
/// @author Jacob Fiedorowicz
namespace lift {

/// @brief Class to adapt a lift driver to the subsystem structure
/// @author Jacob Fiedorowicz
class LiftSubsystem : public ASubsystem {
public:
    /// @brief Construct a new Lift Subsystem object
    /// @param lift_driver __std::unique_ptr<ILift>__ The lift driver class to use by the subsystem
    LiftSubsystem(std::unique_ptr<ILift> lift_driver);

    /// @brief Initializes the lift subsystem
    void init() override;

    /// @brief Runs the lift subsystem
    void run() override;

    /// @brief Send a command to the lift subsystem
    /// @param cmd __const commands::Command&__ The command to send
    void command(const commands::Command& cmd) override;

    /// @brief Retrieves a state for the lift subsystem
    /// @param height __ESubsystemState__ The state to retrieve from the lift subsystem
    /// @return __void*__ The value of the desired state
    void* state(ESubsystemState height) override;

    private:
    std::unique_ptr<ILift> m_lift_driver;

    /// @brief Handles the set lift state command
    /// @param cmd __const commands::lift::SetLiftStateCommand__ The command to handle
    void handleCommand(const commands::lift::SetLiftHeightCommand cmd);

    void handleCommand(const auto cmd);

};
}
}
}
}

#endif
#ifndef __ALIGNER_SUBSYSTEM_HPP__
#define __ALIGNER_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/aligner/IAligner.hpp"
/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the aligner subsystem
/// @author Jacob Fiedorowicz
namespace aligner {

class AlignerSubsystem : public ASubsystem {
    public:
    /// @brief Constructs a new Aligner Subsystem object
    /// @param aligner_driver __std::unique_ptr<IAligner>__ The aligner driver class to use by the subsystem
    AlignerSubsystem(std::unique_ptr<IAligner> aligner_driver);
    
    /// @brief Initializes the aligner subsystem
    void init() override;

    /// @brief Runs the aligner subsystem
    void run() override;

    /// @brief Send a command to the aligner subsystem
    /// @param cmd __const::commands::Command&__ The command to send
    void command(const commands::Command& cmd) override;

    /// @brief Retrieves the current state of the aligner subsystem
    /// @param state __ESubsystemState__ The state to retrieve from the aligner subsystem
    /// @return __void*__ The value of the desired state
    void* state(ESubsystemState state) override;

    private:
    std::unique_ptr<IAligner> m_aligner_driver;

    /// @brief Handles the set aligner state command
    /// @param cmd __const::commands::aligner::SetAlignerStateCommand__ The command to handle
    void handleCommand(const commands::aligner::SetAlignerStateCommand cmd);

    /// @brief Handles the toggle aligner state command
    /// @param cmd __const::commands::aligner::ToggleAlignerStateCommand__ The command to handle
    void handleCommand(const commands::aligner::ToggleAlignerStateCommand cmd);

    /// @brief Handles any commands that are not explicitly handled by the aligner subsystem
    /// @param cmd _const auto__ The command to handle
    void handleCommand(const auto cmd);

};
}
}
}
}

#endif
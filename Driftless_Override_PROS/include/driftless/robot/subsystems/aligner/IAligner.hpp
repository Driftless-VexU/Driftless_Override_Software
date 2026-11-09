#ifndef __I_ALIGNER_HPP__
#define __I_ALIGNER_HPP__

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

class IAligner {
    public:
    /// @brief Destroy the IAligner object
    virtual ~IAligner() = default;

    /// @brief Initialize the IAligner object
    virtual void init() = 0;

    /// @brief Run the IAlinger object
    virtual void run() = 0;

    /// @brief Set the state of the aligner
    /// @param deployed __bool__ Whether the aligner is deployed or not, false for retracted
    virtual void setState(bool deployed) = 0;

    /// @brief Toggles the state of the aligner
    virtual void toggleState() = 0;

    /// @brief Determine if the aligner is deployed
    /// @return __bool__ Whether the brakes are deployed or not, true is deployed, false is retracted
    virtual bool isDeployed() = 0;
};
}
}
}
}

#endif
#ifndef __DIRECT_CONTINUOUS_LIFT_BUILDER_HPP__
#define __DIRECT_CONTINUOUS_LIFT_BUILDER_HPP__

#include "driftless/robot/subsystems/lift/DirectContinuousLift.hpp"

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
/// @author Matthew Backman
namespace lift {

/// @brief Builder for the DirectContinuosuLift Object
/// @author Jacob Fiedorowicz
class DirectContinuousLiftBuilder {
    friend class DirectContinuousLift;

    public:
    /// @brief Adds a motor to the builder
    /// @param motor __std::unique_ptr<io::IMotor>__ The motor to use
    /// @return _DirectContinuousLift&&__ Referncee to the active builder
    DirectContinuousLiftBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

    /// @brief Builds the DirectContinuousLift object
    /// @return __DirectContinuousLift__ the newly constructed DirectContinuousLift
    /// object
    [[nodiscard]]
    DirectContinuousLift build() &&;

    /// @brief Builds a new DirectContinuousLift object wrapped in a unique pointer
    /// @return __std::unique_ptr<DirectContinuousLift>__ Unique pointer to the newly constructed DirectContinuousLift object
    [[nodiscard]]
    std::unique_ptr<DirectContinuousLift> buildUnique() &&;

    private:
    std::unique_ptr<io::IMotor> m_task;
};
}
}
}
}

#endif
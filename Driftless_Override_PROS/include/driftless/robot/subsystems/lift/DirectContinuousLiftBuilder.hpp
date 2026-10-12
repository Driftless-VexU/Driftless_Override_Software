#ifndef __DIRECT_CONTINUOUS_LIFT_BUILDER_HPP__
#define __DIRECT_CONTINUOUS_LIFT_BUILDER_HPP__

#include "driftless/robot/subsystems/lift/DirectContinuousLift.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"

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

    /// @brief Adds a task to the lift
    /// @param task __std::unique_ptr<rtos::ITask>__ The task to use
    /// @return _DirectContinuousLift&&__ A pointer to this builder
    DirectContinuousLiftBuilder&& withTask(std::unique_ptr<rtos::ITask> task) &&;

    /// @brief Adds a delayer to the lift
    /// @param delayer __std::unique_ptr<rtos::IDelayer>__ The delayer to use
    /// @return _DirectContinuousLift&&__ A pointer to this builder
    DirectContinuousLiftBuilder&& withDelayer(std::unique_ptr<rtos::IDelayer> delayer) &&;

    /// @brief Adds a mutex to the lift
    /// @param mutex __std::unique_ptr<rtos::IMutex>__ The mutex to use
    /// @return _DirectContinuousLift&&__ A pointer to this builder
    DirectContinuousLiftBuilder&& withMutex(std::unique_ptr<rtos::IMutex> mutex) &&;

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
    std::unique_ptr<io::IMotor> m_motor{};

    std::unique_ptr<rtos::ITask> m_task{};

    std::unique_ptr<rtos::IDelayer> m_delayer{};

    std::unique_ptr<rtos::IMutex> m_mutex{};
};
}
}
}
}

#endif
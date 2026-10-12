#include "driftless/robot/subsystems/lift/DirectContinuousLiftBuilder.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"

namespace driftless::robot::subsystems::lift {
DirectContinuousLiftBuilder&& DirectContinuousLiftBuilder::withMotor(std::unique_ptr<io::IMotor> motor) && {
    m_motor = std::move(motor);
    return std::move(*this);
}
DirectContinuousLiftBuilder&& DirectContinuousLiftBuilder::withTask(std::unique_ptr<rtos::ITask> task) && {
    m_task = std::move(task);
    return std::move(*this);
}
DirectContinuousLiftBuilder&& DirectContinuousLiftBuilder::withDelayer(std::unique_ptr<rtos::IDelayer> delayer) && {
    m_delayer = std::move(delayer);
    return std::move(*this);
}
DirectContinuousLiftBuilder&& DirectContinuousLiftBuilder::withMutex(std::unique_ptr<rtos::IMutex> mutex) && {
    m_mutex = std::move(mutex);
    return std::move(*this);
}
std::unique_ptr<DirectContinuousLift> DirectContinuousLiftBuilder::buildUnique() &&{
    if(!m_delayer || !m_mutex || !m_task) {
        throw std::runtime_error(
            "One or more RTOS components not set for the DirectContinuousLiftBuilder"
        );
    }
    return std::unique_ptr<DirectContinuousLift>{new DirectContinuousLift(std::move(*this))};
}
DirectContinuousLift DirectContinuousLiftBuilder::build() &&{
    if(!m_delayer || !m_mutex || !m_task) {
        throw std::runtime_error(
            "One or more RTOS components not set for the DirectContinuousLiftBuilder"
        );
    }
    return DirectContinuousLift(std::move(*this));
}
}
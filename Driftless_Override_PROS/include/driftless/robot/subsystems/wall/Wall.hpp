#ifndef __WALL_HPP__
#define __WALL_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/wall/IWall.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the wall subsystem
/// @author Jacob Fiedorowicz
namespace wall {

// builder declaration, see WallBuilder.hpp for definition
class WallBuilder;

/// @brief Wall implementation
/// @author Jacob Fiedorowicz
class Wall : public IWall {
    friend class WallBuilder;

    public:
    /// @brief Initialize the wall object
    void init() override;

    /// @brief Run the wall object
    void run() override;

    /// @brief Sets the state of the wall 
    /// @param deployed __bool__ Wether the wal is deployed or not. True if deployed, false if retracted
    void setWallExtensionState(bool deployed) override;

    /// @brief Toggles the state of the wall
    void toggleWallExtensionState() override;

    /// @brief Determines if the wall is deployed
    /// @return __bool__ Whether the wall is deployed or not. True if deployed, false if retracted
    bool isWallExtended() override;

    /// @brief Sets the state of the wall wheel
    /// @param deployed __bool__ Whether the wall wheel is deployed or not. True if deployed, false if retracted
    void setWallWheelState(bool deployed) override;

    /// @brief Toggles the state of the wall wheel
    void toggleWallWheelState() override;

    /// @brief Sets the speed for the wheel motor
    /// @param voltage __float__ The voltage that the motor has to spin the wheel with
    void setWallWheelMotorVoltage(float voltage) override;

    private:
    std::unique_ptr<io::IPiston> pistonWall;
    std::unique_ptr<io::IPiston> pistonWheel;
    hal::MotorGroup wheelMotor;

    /// @brief Construct a new wall object
    /// @param builder __WallBuilder__ The builder creating this wall object. Consumed on use.
    Wall(WallBuilder&& builder);
};
}
}
}
}

#endif
#ifndef __ALIGNER_OPERATOR_HPP__
#define __ALIGNER_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for aligner control during operator control
/// @author Jacob Fiedorowicz
namespace aligner {

/// @brief Operator class for the aligner subsystem
/// @author Jacob Fiedorowicz
class AlignerOperator {
    public:
    /// @brief Construct a new Aligner Operator object
    /// @param controller __const std::shared_ptr<io::IController>&__ The controller to recieve input from
    /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
    AlignerOperator(const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot);


};
}
}
}

#endif
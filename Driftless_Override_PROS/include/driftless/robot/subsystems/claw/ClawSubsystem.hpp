#ifndef __CLAW_SUBSYSTEM_HPP__
#define __CLAW_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/claw/IClaw.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace containing the claw subsystem
/// @author Matthew Backman
namespace claw {

class ClawSubsystem : public ASubsystem {
public:
ClawSubsystem(std::unique_ptr<IClaw> claw);

void init() override;

void run() override;

void command(const commands::Command& cmd) override;

void* state(const ESubsystemState state_name) override;
private:
  std::unique_ptr<IClaw> m_claw_driver{};
};

}
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
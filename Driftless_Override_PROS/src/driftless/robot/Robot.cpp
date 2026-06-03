#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace robot {
void Robot::addSubsystem(std::unique_ptr<subsystems::ASubsystem>& subsystem) {
  subsystems::ESubsystem subsystem_name{subsystem->getName()};
  subsystems.emplace(subsystem_name, std::move(subsystem));
}

bool Robot::removeSubsystem(subsystems::ESubsystem subsystem) {
  bool removed{false};
  removed = subsystems.erase(subsystem) > 0;
  return removed;
}

void Robot::init() {
  // init all subsystems
  for (auto i{subsystems.begin()}; i != subsystems.end(); ++i) {
    // second gets the value at the address instead of the key
    i->second->init();
  }
}

void Robot::run() {
  // run all subsystems
  for (auto i{subsystems.begin()}; i != subsystems.end(); ++i) {
    // second gets the value at the address instead of the key
    i->second->run();
  }
}

void Robot::sendCommand(subsystems::ESubsystem subsystem_name,
                        const commands::Command command) {
  if (subsystems.contains(subsystem_name)) {
    subsystems.at(subsystem_name)->command(command);
  }
}

void* Robot::getState(subsystems::ESubsystem subsystem_name,
                      subsystems::ESubsystemState state_name) {
  // basic ptr init
  void* state{nullptr};
  // find correct subsystem
  if (subsystems.contains(subsystem_name)) {
    state = subsystems.at(subsystem_name)->state(state_name);
  }
  return state;
}
}  // namespace robot
}  // namespace driftless
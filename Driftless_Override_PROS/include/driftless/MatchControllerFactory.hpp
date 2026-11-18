#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

// general includes
#include "driftless/MatchController.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Factory for creating a MatchController object
/// @author Matthew Backman
class MatchControllerFactory {
 public:
  /// @brief Creates a new MatchController object
  /// @return __driftless::MatchController__ The new MatchController object
  static driftless::MatchController createMatchController();
};
}  // namespace driftless
#endif
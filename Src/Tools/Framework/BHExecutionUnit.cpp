/**
 * @file BHExecutionUnit.cpp
 *
 * This file implements the class \c BHExecutionUnit.
 *
 * @author Arne Hasselbring
 */

#include "BHExecutionUnit.h"
#include "Tools/Framework/BHLoggingController.h"
#include "Debugging/Annotation.h"
#include "Framework/Blackboard.h"
#include "Framework/Configuration.h"
#include "Streaming/TypeRegistry.h"

const LoggingController* BHExecutionUnit::initLogging(const Configuration& config, const std::size_t index)
{
  // If this module container is the (first) in which the GameState is provided, it is the "logging controller".
  // If no thread provides the GameState (e.g. because it is a default representation), it doesn't help anyway.
  // Changing the GameState-providing thread at runtime doesn't make sense either.
  for(const auto& representationProvider : config()[index].representationProviders)
    if(representationProvider.representation == "GameState")
    {
      for(std::size_t i = 0; i < index; ++i)
        for(const auto& otherRepresentationProvider : config()[i].representationProviders)
          if(otherRepresentationProvider.representation == "GameState")
            return nullptr;

      return new BHLoggingController;
    }
  return nullptr;
}

void BHExecutionUnit::beforeModules()
{
  if(Blackboard::getInstance().exists("GameState"))
  {
    const GameState& gameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
    if(gameState.state != lastGameState)
      ANNOTATION("GameState", "Switched to " << TypeRegistry::getEnumName(gameState.state) << " state.");
    lastGameState = gameState.state;
  }
}

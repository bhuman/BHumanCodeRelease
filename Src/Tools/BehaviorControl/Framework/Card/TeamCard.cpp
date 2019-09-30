/**
 * @file TeamCard.cpp
 *
 * This file implements methods for team cards.
 *
 * @author Arne Hasselbring
 */

#include "TeamCard.h"
#include "Platform/BHAssert.h"

thread_local TeamCardRegistry* TeamCardRegistry::theInstance = nullptr;

TeamCardRegistry::TeamCardRegistry(ActivationGraph& activationGraph) :
  CardRegistryBase(activationGraph)
{
  ASSERT(!theInstance);
  theInstance = this;

  create(CardCreatorList<TeamCard>::first);
}

TeamCardRegistry::~TeamCardRegistry()
{
  destroy();

  ASSERT(theInstance == this);
  theInstance = nullptr;
}

void TeamCard::call()
{
  if(_context.lastFrame != TeamCardRegistry::theInstance->lastFrameTime && _context.lastFrame != TeamCardRegistry::theInstance->currentFrameTime)
  {
    _context.behaviorStart = TeamCardRegistry::theInstance->currentFrameTime;
    _context.stateName = nullptr;
    reset();
  }
  ActivationGraph& theActivationGraph = TeamCardRegistry::theInstance->theActivationGraph;
  const size_t activationGraphIndex = theActivationGraph.graph.size();
  theActivationGraph.graph.emplace_back(_name, ++theActivationGraph.currentDepth, "", TeamCardRegistry::theInstance->currentFrameTime - _context.behaviorStart, 0, std::vector<std::string>());
  execute();
  if(_context.stateName) \
  {
    theActivationGraph.graph[activationGraphIndex].state = _context.stateName;
    theActivationGraph.graph[activationGraphIndex].stateTime = TeamCardRegistry::theInstance->currentFrameTime - _context.stateStart;
  }
  --theActivationGraph.currentDepth;
  _context.lastFrame = TeamCardRegistry::theInstance->currentFrameTime;
}

void TeamCard::setState(const char* name)
{
  if(name != _context.stateName)
  {
    _context.stateName = name;
    _context.stateStart = TeamCardRegistry::theInstance->currentFrameTime;
  }
}

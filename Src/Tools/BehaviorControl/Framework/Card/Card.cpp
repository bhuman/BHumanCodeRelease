/**
 * @file Card.cpp
 *
 * This file implements methods for individual robot cards.
 *
 * @author Arne Hasselbring
 */

#include "Card.h"
#include "Platform/BHAssert.h"

thread_local CardRegistry* CardRegistry::theInstance = nullptr;

CardRegistry::CardRegistry(ActivationGraph& activationGraph) :
  CardRegistryBase(activationGraph)
{
  ASSERT(!theInstance);
  theInstance = this;

  create(CardCreatorList<Card>::first);
}

CardRegistry::~CardRegistry()
{
  destroy();

  ASSERT(theInstance == this);
  theInstance = nullptr;
}

void Card::call()
{
  if(_context.lastFrame != CardRegistry::theInstance->lastFrameTime && _context.lastFrame != CardRegistry::theInstance->currentFrameTime)
  {
    _context.behaviorStart = CardRegistry::theInstance->currentFrameTime;
    _context.stateName = nullptr;
    reset();
  }
  ActivationGraph& theActivationGraph = CardRegistry::theInstance->theActivationGraph;
  const size_t activationGraphIndex = theActivationGraph.graph.size();
  theActivationGraph.graph.emplace_back(_name, ++theActivationGraph.currentDepth, "", CardRegistry::theInstance->currentFrameTime - _context.behaviorStart, 0, std::vector<std::string>());
  execute();
  if(_context.stateName) \
  {
    theActivationGraph.graph[activationGraphIndex].state = _context.stateName;
    theActivationGraph.graph[activationGraphIndex].stateTime = CardRegistry::theInstance->currentFrameTime - _context.stateStart;
  }
  --theActivationGraph.currentDepth;
  _context.lastFrame = CardRegistry::theInstance->currentFrameTime;
}

void Card::setState(const char* name)
{
  if(name != _context.stateName)
  {
    _context.stateName = name;
    _context.stateStart = CardRegistry::theInstance->currentFrameTime;
  }
}

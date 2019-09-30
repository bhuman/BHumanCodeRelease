/**
 * @file CardRegistryBase.cpp
 *
 * This file implements a class that manages cards.
 *
 * @author Jesse Richter-Klug
 */

#include "CardRegistryBase.h"
#include "CardBase.h"
#include "CardDetails.h"
#include "Platform/BHAssert.h"

CardRegistryBase::CardRegistryBase(ActivationGraph& activationGraph) :
  theActivationGraph(activationGraph)
{
}

CardRegistryBase::~CardRegistryBase()
{
  ASSERT(cards.empty());
}

void CardRegistryBase::create(CardCreatorBase* firstCreator)
{
  for(CardCreatorBase* card = firstCreator; card != nullptr; card = card->next)
    cards.emplace(std::string(card->name), card->createNew());
}

void CardRegistryBase::destroy()
{
  for(const auto& card : cards)
    delete card.second;
  cards.clear();
}

void CardRegistryBase::modifyAllParameters()
{
  for(auto& card : cards)
    card.second->modifyParameters();
}

void CardRegistryBase::preProcess(unsigned frameTime)
{
  currentFrameTime = frameTime;
  for(auto& card : cards)
    card.second->preProcess();
}

void CardRegistryBase::postProcess()
{
  for(auto& card : cards)
    card.second->postProcess();
  lastFrameTime = currentFrameTime;
}

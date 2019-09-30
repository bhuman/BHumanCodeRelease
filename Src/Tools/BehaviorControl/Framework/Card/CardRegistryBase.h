/**
 * @file CardRegistryBase.h
 *
 * This file declares a class that manages cards.
 * DO NOT INCLUDE THIS FILE DIRECTLY FROM OUTSIDE THIS DIRECTORY!
 *
 * @author Jesse Richter-Klug
 */

#pragma once

#include "Platform/BHAssert.h"
#include <unordered_map>
#include <string>

struct ActivationGraph;
class CardBase;
class CardCreatorBase;

class CardRegistryBase
{
public:
  /**
   * Constructor.
   * @param activationGraph The activation graph that can be modified by cards.
   */
  CardRegistryBase(ActivationGraph& activationGraph);
  /** Destructor (deliberately not virtual). */
  ~CardRegistryBase();

  /**
   * Creates all cards from a creator list and adds them to the registry.
   * @param firstCreator The anchor of the creator list.
   */
  void create(CardCreatorBase* firstCreator);
  /** Destroys all cards in the registry. */
  void destroy();

  /** Calls \c modifyParameters on all cards. */
  void modifyAllParameters();

  /**
   * Calls \c preProcess on all cards and updates the current frame time.
   * @param frameTime The current time from the FrameInfo.
   */
  void preProcess(unsigned frameTime);
  /** Calls \c postProcess on all cards and updates the last frame time. */
  void postProcess();

  /**
   * Obtains a handle to a card.
   * @param cardName The name of the card
   * @return A pointer to the card
   */
  CardBase* getCard(const std::string& cardName)
  {
#ifndef NDEBUG
    if(cards.find(cardName) == cards.end())
      FAIL("Could not find card with name: " << cardName);
#endif
    return cards[cardName];
  }

  ActivationGraph& theActivationGraph; /**< The activation graph that can be modified by cards. */

  unsigned currentFrameTime = 0; /**< The frame time during the current card executions. */
  unsigned lastFrameTime = 0; /**< The frame time during the previous card executions. */

private:
  std::unordered_map<std::string, CardBase*> cards; /**< Instances of all existing cards. */
};

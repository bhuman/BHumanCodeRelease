/**
 * @file Dealer.h
 *
 * This file declares a struct which represents a deck of cards
 * and classes which deal a card from a deck according to some dealing method.
 *
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#pragma once

#include "CardBase.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/AutoStreamable.h"
#include <limits>
#include <string>
#include <vector>

template<typename Registry>
STREAMABLE(DeckOfCards,
{
  /**
   * Accesses a card by its index.
   * @param i The index in the deck.
   * @return A pointer to the card at that index.
   */
  CardBase* operator[](size_t i) const
  {
    // TODO: Do not call getCard every time because it is a map access.
    // (but prefetching the cards in onRead does not work because it is executed at construction time of cards,
    //  which means that other cards may not have been constructed yet).
    return Registry::theInstance->getCard(cards[i]);
  }
  /**
   * Checks whether a card is in this deck.
   * @param card The pointer to the card to check.
   * @return Whether the card is in this deck.
   */
  bool contains(CardBase* card) const
  {
    for(size_t i = 0; i < cards.size(); ++i)
      if((*this)[i] == card)
        return true;
    return false;
  },

  (bool) sticky, /**< Whether the previously selected card should stay selected if it is still playable. */
  (std::vector<std::string>) cards, /**< A list of card names that are in this deck. */
});

/**
 * This dealer views the deck as a priority ordered list and deals the card with the highest priority.
 * Cards with lower priority than the previously dealt one will only be dealt after its postconditions have become true.
 */
class PriorityListDealer
{
public:
  /**
   * Deals a card from the deck. The first card with true preconditions is selected, unless
   * it comes after the previously dealt card and its postconditions are not yet true.
   * @tparam Registry The registry from which to obtain cards.
   * @param deck The deck from which to select a card.
   * @return The selected card.
   */
  template<typename Registry>
  CardBase* deal(const DeckOfCards<Registry>& deck)
  {
    CardBase* nextCard = nullptr;
    if(deck.sticky && lastCard && deck.contains(lastCard) && !lastCard->postconditions())
      return lastCard;
    for(size_t i = 0; i < deck.cards.size(); ++i)
    {
      CardBase* card = deck[i];
      if((card == lastCard) ? (!card->postconditions() || card->preconditions()) : card->preconditions())
      {
        nextCard = card;
        break;
      }
    }
    ASSERT(nextCard);
    lastCard = nextCard;
    return nextCard;
  }
  /** Lets the dealer forget the previously dealt card. */
  void reset()
  {
    lastCard = nullptr;
  }
private:
  CardBase* lastCard = nullptr; /**< The previously dealt card. */
};

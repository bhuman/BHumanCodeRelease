/**
 * @file CardBase.h
 *
 * This file declares the base class for cards.
 * DO NOT INCLUDE THIS FILE DIRECTLY FROM OUTSIDE THIS DIRECTORY!
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Framework/BehaviorContext.h"

class CardBase
{
public:
  /**
   * Constructor.
   * @param name The name of the derived card.
   */
  CardBase(const char* name) :
    _name(name)
  {}
  /** Virtual destructor for polymorphism. */
  virtual ~CardBase() = default;
  /** Indicates whether this card may be entered. */
  virtual bool preconditions() const { return false; }
  /** Indicates whether this card may/must be left. */
  virtual bool postconditions() const { return !preconditions(); }
  /** Calls the card (i.e. adds it to the activation graph, calls \c reset if needed, calls \c execute). */
  virtual void call() = 0;

protected:
  /** Executes the card. */
  virtual void execute() = 0;
  /** Resets the card. Is called if before \c execute if this card was not called in the last frame. */
  virtual void reset() {}
  mutable BehaviorContext _context; /**< The behavior context of this card. */
  const char* _name; /**< The name of the derived card (for the ActivationGraph). */
private:
  /** Is called each frame before the behavior has been run. */
  virtual void preProcess() {}
  /** Is called each frame after the behavior has been run. */
  virtual void postProcess() {}
  /** Calls \c MODIFY on the parameters of the card. */
  virtual void modifyParameters() = 0;
  friend class CardRegistryBase;
};

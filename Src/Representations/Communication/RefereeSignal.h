/**
 * @file RefereeSignal.h
 *
 * This file defines a representation that represents the referee signal that should be sent to
 * the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Perception/RefereePercept/RefereePercept.h"

STREAMABLE(RefereeSignal,
{,
  (RefereePercept::Gesture)(RefereePercept::none) signal,
  (unsigned)(0) timeWhenWhistleWasHeard,
  (unsigned)(0) timeWhenLastDetected,
});

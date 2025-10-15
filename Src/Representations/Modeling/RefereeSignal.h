/**
 * @file RefereeSignal.h
 *
 * This file declares a representation that represents a referee signal.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Perception/RefereeGestures/RefereeGesture.h"
#include "Tools/Communication/BHumanMessageParticle.h"

STREAMABLE(RefereeSignal, COMMA BHumanCompressedMessageParticle<RefereeSignal>
{
  using enum RefereeGesture::Gesture;
  using Signal = RefereeGesture::Gesture;
  static const Signal numOfSignals = numOfGestures,

  (Signal)(none) signal, /**< The last signal that was detected. */
  (unsigned)(0) timeWhenDetected, /**< When it was detected (in ms). */
});

#pragma once

#include "Streaming/Enum.h"
#include "Tools/Communication/BHumanMessageParticle.h"

/**
 * @struct InitialToReady
 * This struct contains information needed to determine, if the transition from initial state to ready state is happening.
 */
STREAMABLE(InitialToReady, COMMA BHumanCompressedMessageParticle<InitialToReady>
{
  ENUM(State,
  {,
    observing,
    waiting,
    transition,
  });

  /**
   * @return true, when transition from initial to ready can be initiated
   */
  bool isTransition() const,

  (State)(observing) state, /**< current state of the robot regarding initial to ready transition */
  (bool)(false) gestureDetected, /**< has this robot detect the referee gesture */
  (int)(0) detectedBy, /**< playerNumber of robot who detected the referee gesture */
  (bool)(false) isPawn, /**< is this robot currently an active pawn */
  (unsigned)(0) timestamp, /**< timestamp from the latest gesture detection or during transition: timestamp of the caused detection */
});

inline bool InitialToReady::isTransition() const
{
  return state == State::transition;
}

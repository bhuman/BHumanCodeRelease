/**
* @file LocalizationTeamBall.h
*
* Declaration of class LocalizationTeamBall
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class LocalizationTeamBall
*
* A ball model based on observations of reliable teammates.
*/
STREAMABLE(LocalizationTeamBall,
{
public:
  void draw() const,

  (Vector2<>) position,                     /**< The position of the ball in the global frame of reference (in mm) */
  (bool) isValid,                           /**< Flag that indicates, whether or not the information is valid */
  (float) ballStateOthersMaxSideConfidence, /**< Confidence indicator, might become replaced soon */
  (int)(0)      numOfObservers,             /**< The number of observations that have been clustered */
  (bool)(false) goalieHasObserved,          /**< The own goalie was among the observers */
  (unsigned)(0) lastObservation,            /**< The point of time when the ball was observed the last time */
});

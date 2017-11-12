/**
 * @file Representations/BehaviorControl/TimeToReachBall.h
 *
 * Representation of the estimated time to reach the ball
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include <limits>
#include <cmath>

/**
 * @struct TimeToReachBall
 * Representation of the Time to reach the ball
 */
STREAMABLE(TimeToReachBall, COMMA public BHumanMessageParticle<undefined>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override,

  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBall,         /**< The estimated time when reach the ball */
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBallStriker,
});

inline void TimeToReachBall::operator >> (BHumanMessage& m) const
{
  m.theBHULKsStandardMessage.timeWhenReachBall = timeWhenReachBall;
  m.theBHULKsStandardMessage.timeWhenReachBallQueen = timeWhenReachBallStriker;
}

inline void TimeToReachBall::operator << (const BHumanMessage& m)
{
  timeWhenReachBall = m.toLocalTimestamp(m.theBHULKsStandardMessage.timeWhenReachBall);
  timeWhenReachBallStriker = m.toLocalTimestamp(m.theBHULKsStandardMessage.timeWhenReachBallQueen);
}

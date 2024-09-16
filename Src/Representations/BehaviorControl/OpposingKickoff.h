/**
 * @file OpposingKickoff.h
 *
 * This file defines a representation that detects the opponents kickoff.
 *
 * @author Moritz Oppermann
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
#include "Debugging/DebugDrawings.h"

STREAMABLE(OpposingKickoff,
{
  ENUM(KickoffVariation,
  {,
    unknown, /**< Kickoff can not be defined. */
    dribble,
    kick,
    nothing,
  });

  inline void draw() const,

  (bool)(false) kickoffHappened, /**< Whether the kickoff already happened. */
  (Vector2f) ballLocation, /**< The last ball location after the opposing kickoff in field coordinates. */
  (std::vector<Vector2f>)({}) ballLocations, /**< The previous ball locations in field coordinates. */
  (std::vector<Vector2f>)({}) clusterMeans, /**< The mean of every cluster. */
  (float)(0.f) absKickYMean, /**< Absolute mean of y-coordinates of previous kick kickoffs. */
  (std::vector<float>)({}) clusterRadii, /**< The radius of every cluster. */
  (std::array<unsigned, numOfKickoffVariations>)({}) numOfVariations, /**< The number of times each kickoff variation was used. */
  (std::vector<KickoffVariation>)({}) lastVariations, /**< The last kickoff variations the opponent used. */
  (KickoffVariation)(unknown) kickoffPrediction, /**< Prediction on what kickoff variation the opponent might use next. */
});

void OpposingKickoff::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:OpposingKickoff:field", "drawingOnField");
  for(const Vector2f& previousBallLocation : ballLocations)
  {
    CIRCLE("representation:OpposingKickoff:field",
           previousBallLocation.x(),
           previousBallLocation.y(),
           50,
           0,
           Drawings::solidPen,
           ColorRGBA::orange,
           Drawings::solidBrush,
           ColorRGBA::orange);
  }
  for(std::size_t i = 0; i < clusterMeans.size(); i++)
  {
    CIRCLE("representation:OpposingKickoff:field",
           clusterMeans[i].x(),
           clusterMeans[i].y(),
           clusterRadii[i] + 25,
           5,
           Drawings::solidPen,
           ColorRGBA::blue,
           Drawings::noBrush,
           ColorRGBA::blue);
  }
}

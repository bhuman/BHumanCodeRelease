/**
 * @file Defender.cpp
 *
 * This file implements the Defender role.
 * Tries to cover as much space as possible by staying far away from the teammates and the field border while not deviate to much from the base pose.
 * It tries to maximize a rating function that is manly based on the distance to the closest teammate.
 *
 * @author Yannik Meinken
 */

#include "Defender.h"
#include "Math/Geometry.h"

void Defender::preProcess()
{
  RatingRole::preProcess();
  DECLARE_DEBUG_DRAWING("behavior:Defender:communicatedPosition", "drawingOnField");
  MODIFY("behavior:Defender", p);

  //override parameters of the base class
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

float Defender::rating(const Vector2f& pos) const
{
  //outside the Voronoi region, the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //get distance to next field border
  const float borderDistance = std::min(std::max(0.f, theFieldDimensions.xPosOpponentGoalLine - std::abs(pos.x())),
                                        std::max(0.f, theFieldDimensions.yPosLeftTouchline - std::abs(pos.y())));

  //better rating far away from the field border
  const float borderRating = 1.f - std::exp(-0.5f * sqr(borderDistance) / sqr(p.sigmaBorder));

  //get the rating based on the nearest teammate
  float rating = 1;
  if(!theGlobalTeammatesModel.teammates.empty())
  {
    auto firstTeammate = theGlobalTeammatesModel.teammates.cbegin();
    float minTeammateDistanceSquared = (pos - firstTeammate->pose.translation).squaredNorm();

    for(auto t = ++theGlobalTeammatesModel.teammates.cbegin(); t != theGlobalTeammatesModel.teammates.cend(); t++)
    {
      const float d = (pos - t->pose.translation).squaredNorm();
      if(d < minTeammateDistanceSquared)
        minTeammateDistanceSquared = d;
    }

    //better rating far away from teammates
    rating = 1.f - std::exp(-0.5f * minTeammateDistanceSquared / sqr(p.sigmaTeam));
  }

  // rating based on the distance to the base pose. Closer to base pose is better
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));

  // get the rating based on potentially marked opponents
  float markRating = p.minMarkRating;
  for(const auto& opponent : theGlobalOpponentsModel.opponents)
  {
    const Vector2f gloObstacleToBlock = opponent.position;

    // compute the 'ideal' position to mark this opponent
    const Vector2f markPosition =
      gloObstacleToBlock + (Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f) - gloObstacleToBlock).normalized(p.distToMarkedRobot);

    // rating based on the distance to the ideal mark position for this opponent
    float localMarkRating = (1.f - p.minMarkRating) * std::exp(-0.5f * (pos - markPosition).squaredNorm() / sqr(p.sigmaMark));
    localMarkRating *= (1 - theExpectedGoals.getOpponentRating(opponent.position)); // higher rating if the opponent is a good position to score

    // combine ratings for all opponents. It could potentially add up to above 1
    markRating += localMarkRating;
  }
  // clip mark rating as it could be added up to above 1
  markRating = std::min(1.f, markRating);

  // rating based on the distance to the line between ball and own goal
  float goalLineRating = p.minGoalLineRating;
  const Vector2f ballOnField = theFieldBall.recentBallPositionOnField();
  Geometry::Line ballToGoal = Geometry::Line(ballOnField, Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f) - ballOnField);

  // only positions between the ball and goal are valid
  if(pos.x() < ballOnField.x())
  {
    const float distanceBallToGoalLine = Geometry::getDistanceToLine(ballToGoal, pos);
    goalLineRating += (1 - p.minGoalLineRating) * std::exp(-0.5f * sqr(distanceBallToGoalLine) / sqr(p.sigmaBallLine));
  }

  // Positions near the last communicated target pose are better
  const Vector2f lastTargetInWorld = agent.lastKnownPose * agent.lastKnownTarget; // Transform from relative to global Coordinates
  const float communicationRating = p.minCommunicationRating +
                                    (1.f - p.minCommunicationRating) * std::exp(-0.5f * (pos - lastTargetInWorld).squaredNorm() / sqr(p.sigmaCommunication));
  CROSS("behavior:Defender:communicatedPosition", lastTargetInWorld.x(), lastTargetInWorld.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);

  //combine the ratings
  return rating * baseRating * borderRating * markRating * goalLineRating * communicationRating;
}

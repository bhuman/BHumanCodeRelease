/**
 * @file AttackingGoalkeeper.cpp
 *
 * This file implements the attacking goalkeeper role.
 *
 * @author Jonas Hotzan, Yannis Meyer
 */

#include "AttackingGoalkeeper.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Platform/SystemCall.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Modify.h"

//Defines the parameters and debug drawings for the goalkeeper
void AttackingGoalkeeper::preProcess()
{
  RatingRole::preProcess();

  MODIFY("behavior:AttackingGoalkeeper", p);
  DECLARE_DEBUG_DRAWING("behavior:AttackingGoalkeeper:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:AttackingGoalkeeper:communicatedPosition", "drawingOnField");

  //override RatingRole Parameters
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

//Describes the attacking behavior of the goalkeeper
float AttackingGoalkeeper::rating(const Vector2f& pos) const
{
  //outside the Voronoi region, the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose via Gaussian function
  const Vector2f meanVector(base.x(), base.y());
  Matrix2f baseCovarianceMatrix;
  baseCovarianceMatrix << p.sigmaBaseX* p.sigmaBaseX, 0, 0, p.sigmaBaseY* p.sigmaBaseY;
  const Vector2f deviation = pos - meanVector;
  const float baseRating = std::exp(-0.5f * deviation.transpose() * baseCovarianceMatrix.inverse() * deviation);

//rating the position based on the difference to the last opponent
  float opponentRating = 1.f;
  if(!(theGlobalOpponentsModel.opponents.empty()))
  {
   auto lastOpponent = theGlobalOpponentsModel.opponents.at(0);

   //calculate the last opponent
   for(const auto& opponent : theGlobalOpponentsModel.opponents)
   {
     if(opponent.position.x() < lastOpponent.position.x())
     {
      lastOpponent = opponent;
     }
   }
   //calculate rating via logistic function
   float diff = lastOpponent.position.x() - pos.x();
   opponentRating = 1.f / (1.f + std::exp(-diff / p.sigmaOpponent));
  }

  //rating for the angle between ball and goalkeeper
  float angRating = angleRating(pos);

  //rating position based on distance to teammates
  float teammateRating = 1;
  if(!theGlobalTeammatesModel.teammates.empty())
  {
    auto firstNonGoalkeeper = theGlobalTeammatesModel.teammates.cbegin();
    if(theGameState.ownTeam.isGoalkeeper(firstNonGoalkeeper->playerNumber))
      firstNonGoalkeeper++;
    float minTeammateDistanceSquared = (pos - firstNonGoalkeeper->pose.translation).squaredNorm();

    for(auto t = ++theGlobalTeammatesModel.teammates.cbegin(); t != theGlobalTeammatesModel.teammates.cend(); t++)
    {
      if(theGameState.ownTeam.isGoalkeeper(t->playerNumber))
        continue;

      const float d = (pos - t->pose.translation).squaredNorm();
      if(d < minTeammateDistanceSquared)
        minTeammateDistanceSquared = d;
    }

    //better rating far away from teammates
    teammateRating = 1.f - std::exp(-0.5f * minTeammateDistanceSquared / sqr(p.sigmaTeammate));
  }

  //Positions near the last communicated target pose are better
  const Vector2f lastTargetInWorld = agent.lastKnownPose * agent.lastKnownTarget; //Transform from relative to global Coordinates
  const float communicationRating = p.minCommunicationRating + (1.f - p.minCommunicationRating) * std::exp(-0.5f * (pos - lastTargetInWorld).squaredNorm() / sqr(p.sigmaCommunication));
  CROSS("behavior:AttackingGoalkeeper:communicatedPosition", lastTargetInWorld.x(), lastTargetInWorld.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);

  //combine all ratings
  return baseRating * opponentRating * angRating * teammateRating * communicationRating;
}

//Calculating the rating for the angle between ball and the own goal
float AttackingGoalkeeper::angleRating(const Vector2f& position) const
{
  Vector2f ballPose = theFieldBall.recentBallPositionOnField();
  if(ballPose.x() < theFieldDimensions.xPosOwnGoalLine)
  {
    ballPose.x() = theFieldDimensions.xPosOwnGoalLine;
  }

  const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
  const Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  const Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - theFieldBall.recentBallPositionOnField()).norm()));
  const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - theFieldBall.recentBallPositionOnField()).norm()));
  const Angle angleToLeftPost = (leftGoalPost - ballPose).angle() - leftAngleOffset;
  const Angle angleToRightPost = (rightGoalPost - ballPose).angle() + rightAngleOffset;

  //Construct a sector wheel relative to the position and add the goal sector between the two goal posts
  SectorWheel wheel;
  std::list<SectorWheel::Sector> sectors;
  Angle openingAngle = 0_deg;
  wheel.begin(ballPose);
  wheel.addSector(Rangea(angleToLeftPost, angleToRightPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

  //Check if the goalkeeper is inside of the goal sector
  const float width = 140.f;
  const float distance = (position - ballPose).norm();
  const float radius = std::atan(width / (2.f * distance));
  const Angle direction = (position - ballPose).angle();

  //Add the goalkeeper to the sector wheel
  wheel.addSector(Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius)), distance, SectorWheel::Sector::obstacle);

  sectors = wheel.finish();
  //Find the maximum opening angle i.e. the free sector with the largest size
  for(const SectorWheel::Sector& sector : sectors)
    if(sector.type == SectorWheel::Sector::goal &&
       sector.angleRange.getSize() >= openingAngle)
      openingAngle = sector.angleRange.getSize();

  //Drawing for the sector wheel
  DRAW_SECTOR_WHEEL("behavior:AttackingGoalkeeper:wheel", sectors, ballPose);

  //return the rating via Gaussian function
  return std::exp(-0.5f * sqr(openingAngle) / sqr(p.sigmaAngle));
}

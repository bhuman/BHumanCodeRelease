/**
 * @file PlayBall.cpp
 *
 * This file implements the default ball playing behavior that selects the action to be executed.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "Debugging/Modify.h"
#include "Modules/BehaviorControl/ActionRatingProvider/ClearTargetProvider.h"
#include "Modules/BehaviorControl/StrategyBehaviorControl/Behavior.h"
#include "PlayBall.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"

void PlayBall::reset()
{
  lastPassTarget = 0;
}

void PlayBall::preProcess()
{
  MODIFY("behavior:PlayBall", p);
  DECLARE_DEBUG_DRAWING("behavior:PlayBall:ratings", "drawingOnField");
}

SkillRequest PlayBall::execute(const Agent& self, const Agents& teammates)
{
  return smashOrPass(self, teammates);
}

SkillRequest PlayBall::smashOrPass(const Agent& self, const Agents& teammates)
{
  const bool allowDirectKick = !Global::getSettings().scenario.starts_with("SharedAutonomyDefender") && theIndirectKick.allowDirectKick;

  if(p.alwaysShoot || (Global::getSettings().scenario.starts_with("SharedAutonomy") && allowDirectKick))
    return SkillRequest::Builder::shoot();

  // Calculate a penalty for changing the decision of the shoot or pass target based on the distance to the ball and the penalty value range
  decisionPenalty = mapToRange(self.ballPosition.norm(), p.ballDistMaxPenalty, p.ballDistMinPenalty, p.maxPenalty, p.minPenalty);
  // Get the estimated probability of shooting a goal from the current ball position
  const Vector2f ballPosition = self.pose * self.ballPosition;
  // No goal shots allowed if indirect kick rule applies
  float bestActionRating = !allowDirectKick ? p.minRating : theExpectedGoals.getRating(ballPosition, false);
  float maxPassDistance = p.maxPassDistance;
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    // Minimize the own goal rating because the rules do not allow this robot to perform a direct kick.
    bestActionRating = p.minRating;
    maxPassDistance = theGameState.isGoalKick() ? p.maxGoalKickDistance : p.maxFreeKickDistance;
  }
  // Apply the decision penalty when this action is different from the last frame
  if(lastPassTarget > 0)
    bestActionRating -= decisionPenalty;
  if(bestActionRating > p.shootThreshold && allowDirectKick)
  {
    lastPassTarget = -1;
    return SkillRequest::Builder::shoot();
  }
  dribbleTarget = theDribbleTarget.getTarget(theFieldInterceptBall.interceptedEndPositionOnField);
  if(allowDirectKick && !(theGameState.isFreeKick() && theGameState.isForOwnTeam()))
  {
    float newBestActionRating = theExpectedGoals.getRating(dribbleTarget, false);
    if(lastPassTarget > 0)
      newBestActionRating -= decisionPenalty;
    bestActionRating = std::max(bestActionRating, newBestActionRating);
    if(bestActionRating > p.shootThreshold)
    {
      lastPassTarget = -1;
      return SkillRequest::Builder::shoot();
    }
    else // Get the rating for a second dribble kick. But only call shoot, if this rating would be better then passing
    {
      const Vector2f secondTarget = theDribbleTarget.getTarget(dribbleTarget);
      float newBestActionRating = theExpectedGoals.getRating(secondTarget, false);
      if(lastPassTarget > 0)
        newBestActionRating -= decisionPenalty;
      bestActionRating = std::max(bestActionRating, newBestActionRating);
    }
  }

  const Agent* passTarget = nullptr;
  // TODO: Should the search start with ballPosition or recentBallBallPositionOnField?
  findPassTarget(theFieldBall.recentBallPositionOnField(), teammates, maxPassDistance, p.maxSearchDepth, passTarget, bestActionRating, decisionPenalty, {});
  if(passTarget)
  {
    lastPassTarget = passTarget->number;
    return SkillRequest::Builder::passTo(passTarget->number);
  }
  else if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    //clear the ball when there is no passtarget
    //when clearing the ball is not possible dribble by calling the pass skill
    if(theClearTarget.getKickType() != KickInfo::numOfKickTypes)
    {
      return SkillRequest::Builder::clear();
    }
    else
    {
      // Here, the pass skill request is set with an invalid pass target on purpose. This way, the waiting states of the pass skill are utilized during setplays. Outside of setplays, the shoot skill request is set and the robot will dribble immediately without waiting behind the ball.
      lastPassTarget = -1;
      return SkillRequest::Builder::passTo(0);
    }
  }
  else
  {
    const float clearRating = theClearTarget.getRating();
    const float dribbleRating = theExpectedGoals.getRating(dribbleTarget, false);
    lastPassTarget = bestActionRating > p.minRating ? -1 : 0;
    if(allowDirectKick)
    {
      float shootRating = theExpectedGoals.getRating(ballPosition, false);
      if(shootRating >= clearRating + clearDribbleHysteresis)
      {
        return SkillRequest::Builder::shoot();
      }
    }
    if(Global::getSettings().scenario.starts_with("SharedAutonomy") || (dribbleRating > clearRating + clearDribbleHysteresis) || theClearTarget.getKickType() == KickInfo::numOfKickTypes)
    {
      // If goalkeeper, just clear the ball
      if(Tactic::Position::isGoalkeeper(self.position) && theClearTarget.getKickType() != KickInfo::numOfKickTypes)
        return SkillRequest::Builder::clear();

      clearDribbleHysteresis = -p.clearDribbleHysteresisValue;
      return SkillRequest::Builder::dribbleTo((dribbleTarget - theRobotPose.translation).angle());
    }
    else
    {
      clearDribbleHysteresis = p.clearDribbleHysteresisValue;
      return SkillRequest::Builder::clear();
    }
  }
}

float PlayBall::findPassTarget(const Vector2f& ballPosition, const Agents& teammates, const float maxPassDistance, int remainingSearchDepth, const Agent*& passTarget, float& bestActionRating, const float decisionPenalty, const std::vector<int>& ballPossessionList)
{
  const bool allowDirectKick = !Global::getSettings().scenario.starts_with("SharedAutonomyDefender") && theIndirectKick.allowDirectKick;
  if(Global::getSettings().scenario.starts_with("SharedAutonomy"))
  {
    for(const Agent* teammate : teammates)
    {
      // when the distance to the teammate is enough to count as a point, always pass to the teammate
      if((teammate->pose.translation - theFieldBall.recentBallPositionOnField()).squaredNorm() > sqr(p.minPassDistanceSAC) && !allowDirectKick)
      {
        passTarget = teammate;
        return 1.f;
      }
    }
    return 0.f;
  }
  if(remainingSearchDepth <= 0)
    return 0.f;

  float bestCombinedRating = 0.f;
  // Find a pass target i.e. the teammate with the highest rating for a successful pass (from the current ball position) and a following goal shot or another pass recursively (from the teammate's current position)
  for(const Agent* teammate : teammates)
  {
    if(teammate->position == Tactic::Position::Type::goalkeeper || std::count(ballPossessionList.begin(), ballPossessionList.end(), teammate->number))
      continue;

    const Vector2f& passTargetPosition = teammate->currentPosition;
    const float passDistance = (passTargetPosition - ballPosition).squaredNorm();
    if(passDistance < sqr(p.minPassDistance) ||
       passDistance > sqr(maxPassDistance))
      continue;

    // Evaluation for the pass
    const float passRating = thePassEvaluation.getRating(ballPosition, passTargetPosition, false);
    // Pruning, if path can't be better than best rating
    if(passRating < bestActionRating)
      continue;

    // Evaluation for a goal
    const float goalRating = theExpectedGoals.getRating(passTargetPosition, false) * theExpectedGoals.getOpponentRating(passTargetPosition);

    // Add current player to list, which tracks who had the ball
    std::vector<int> currentBallPossessionList = ballPossessionList;
    currentBallPossessionList.push_back(teammate->number);

    // The teammate's rating is the estimated probability that the ball would reach the given target position when passed AND the teammate would score a direct goal from the pass target position
    const float nextRating = p.discountFactor * findPassTarget(passTargetPosition, teammates, maxPassDistance, remainingSearchDepth - 1, passTarget, bestActionRating, decisionPenalty, currentBallPossessionList);
    float combinedRating = passRating * std::max(goalRating, nextRating);

    draw(ballPosition, passTargetPosition, combinedRating, 0.f, false);

    if(remainingSearchDepth == p.maxSearchDepth)
    {
      // Apply the decision penalty when this action is different from the last frame
      const bool applyDecisionPenalty = lastPassTarget > 0 && lastPassTarget != teammate->number;

      if(applyDecisionPenalty)
        combinedRating -= decisionPenalty;
      if(combinedRating > bestActionRating)
      {
        bestActionRating = combinedRating;
        passTarget = teammate;
      }
    }
    if(combinedRating > bestCombinedRating)
      bestCombinedRating = combinedRating;
  }
  return bestCombinedRating;
}

void PlayBall::draw([[maybe_unused]] const Vector2f& passBasePosition, [[maybe_unused]] const Vector2f& passTargetPosition, [[maybe_unused]] const float value, [[maybe_unused]] const float penalty, const bool applyPenalty)
{
  COMPLEX_DRAWING("behavior:PlayBall:ratings")
  {
    const bool allowDirectKick = !Global::getSettings().scenario.starts_with("SharedAutonomyDefender") && theIndirectKick.allowDirectKick;

    float maxPassDistance = (theGameState.isFreeKick() && theGameState.isForOwnTeam()) || !allowDirectKick ?
                            (theGameState.isGoalKick() ? p.maxGoalKickDistance : p.maxFreeKickDistance) :
                            p.maxPassDistance;
    CIRCLE("behavior:PlayBall:ratings", passBasePosition.x(), passBasePosition.y(), maxPassDistance, 20, Drawings::solidPen, ColorRGBA::violet, Drawings::noPen, ColorRGBA::violet);
    LINE("behavior:PlayBall:ratings", passBasePosition.x(), passBasePosition.y(), passTargetPosition.x(), passTargetPosition.y(), 20, Drawings::dashedPen, applyPenalty ? ColorRGBA::violet : ColorRGBA::blue);
    DRAW_TEXT("behavior:PlayBall:ratings", passTargetPosition.x(), passTargetPosition.y(), 250, ColorRGBA::blue, value);
    if(applyPenalty)
      DRAW_TEXT("behavior:PlayBall:ratings", passTargetPosition.x(), passTargetPosition.y() - 250, 250, ColorRGBA::violet, value - penalty);
  }
}

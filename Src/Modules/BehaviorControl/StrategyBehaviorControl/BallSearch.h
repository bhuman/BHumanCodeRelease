/**
 * @file BallSearch.h
 *
 * This file declares a ball search behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Debugging/Annotation.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Tools/BehaviorControl/Strategy/BehaviorBase.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Math/Geometry.h"
#include <array>
#include <regex> // not needed in header, but would otherwise be broken by CABSL

class BallSearch;

#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX BallSearch::
#endif
#include "Tools/BehaviorControl/Cabsl.h"

class BallSearch : public cabsl::Cabsl<BallSearch>, public BehaviorBase
{
public:
  BallSearch();

  void preProcess() override;

  void postProcess() override;

  SkillRequest execute(const Agent& agent, const Agents& otherAgents);

private:
  ActivationGraph activationGraph;
  SkillRequest skillRequest; /**< Skill request for ballSearch behavior */
  Agents agents;
  const float goalLineXOffset = 50.f;
  const float minRadius = 20.f;
  float initialRadius;
  const Agent* agent;

  const int lastBallPositionThreshold = 6000; /**< if the last known ball position was longer that this not in view at the beginning of the search check it first. */
  const float ignoreVoronoiThreshold = 1500.f; /**< if the last known ball position is closer that this check it first even if it is outside the Voronoi cell */

  option(Root)
  {
    const Pose2f goalCenterOnFieldWithOffset = Pose2f(0.f, theFieldDimensions.xPosOwnGoalLine + goalLineXOffset, 0.f);
    const Pose2f goalCenterRelativeWithOffset = theRobotPose.inverse() * goalCenterOnFieldWithOffset;

    const auto getBallCell = [&]
    {
      // Todo: refactor the grid to find positions by indices
      // find the cell next to the ball
      auto cellsToSearch = theBallSearchAreas.grid;
      return *std::min_element(cellsToSearch.cbegin(), cellsToSearch.cend(), [&](const BallSearchAreas::Cell& a, const BallSearchAreas::Cell& b)
      {
        return (a.positionOnField - theFieldBall.recentBallPositionOnField()).squaredNorm() <
               (b.positionOnField - theFieldBall.recentBallPositionOnField()).squaredNorm();
      });
    };

    /**
     * return true if the condition to first check the last ball position is met
     */
    const auto checkNearLastBallCondition = [&](const BallSearchAreas::Cell nextCellToBall)
    {
      const bool longerNotChecked = theFrameInfo.getTimeSince(nextCellToBall.timestamp) > lastBallPositionThreshold;
      const bool notToFar = (theFieldBall.recentBallPositionOnField() - agent->pose.translation).norm() < ignoreVoronoiThreshold ||
          Geometry::isPointInsideConvexPolygon(agent->baseArea.data(), static_cast<int>(agent->baseArea.size()), theFieldBall.recentBallPositionOnField());
      const bool notMovedByGameState = !theGameState.isFreeKick() || theGameState.isPushingFreeKick();

      return longerNotChecked && notToFar && notMovedByGameState;
    };

    //the initial ballSearch
    initial_state(initial)
    {
      transition
      {
        ANNOTATION("BallSearch", "is active");
        if(agent->isGoalkeeper)
          goto goalkeeper;
        else
        {
          if(!theBallSearchAreas.grid.empty() && checkNearLastBallCondition(getBallCell()))
            goto checkNearLastBall;
          else
            goto gridSearch;
        }
      }
    }

    // first look near the last position of the ball
    state(checkNearLastBall)
    {
      BallSearchAreas::Cell nextCellToBall = getBallCell();
      transition
      {
        if(!checkNearLastBallCondition(nextCellToBall))
        {
          if(agent->isGoalkeeper)
            goto goalkeeper;
          else
            goto gridSearch;
        }
      }
      action
      {
        skillRequest = SkillRequest::Builder::observe(nextCellToBall.positionOnField);
      }
    }

    // if the ball is lost in a non-standard situation, the robot will use the ballSearchAreas Grid to search the ball.
    state(gridSearch)
    {
      transition
      {
        if(agent->isGoalkeeper)
        {
          goto goalkeeper;
        }
      }
      action
      {
        skillRequest = SkillRequest::Builder::observe(theBallSearchAreas.cellToSearchNext(*agent));
      }
    }

    //BallSearch Behavior for the Goalkeeper
    state(goalkeeper)
    {
      transition
      {
        {
          if((theGameState.isCornerKick() || theGameState.isGoalKick()) && theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) < 10000)
            goto goalkeeperStandardSituation;
          else if(theFieldBall.timeSinceBallWasSeen > 10000 && theRobotPose.translation.x() < theFieldDimensions.xPosOwnGoalLine)
            goto goalkeeperWalkToTarget;
          else if(theFieldBall.timeSinceBallWasSeen > 10000 && (std::abs(goalCenterRelativeWithOffset.translation.angle()) > 45_deg))
          {
            initialRadius = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f)).norm();;
            goto goalkeeperAvoidBall;
          }
          else if(theFieldBall.timeSinceBallWasSeen > 10000)
            goto goalkeeperTurnToTarget;
        }
      }
      action
      {
        skillRequest = SkillRequest::Builder::walkTo(agent->basePose);
      }
    }

    // during a standard situation the goalkeeper will remain at the current position
    state(goalkeeperStandardSituation)
    {
      transition
      {
        if((!theGameState.isCornerKick() && !theGameState.isGoalKick()) || theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > 10000)
        {
          goto goalkeeper;
        }
      }
      action
      {
        skillRequest = SkillRequest::Builder::stand();
      }
    }

    // the goalkeeper will walk to a given position
    state(goalkeeperWalkToTarget)
    {
      action
      {
        skillRequest = SkillRequest::Builder::walkTo(goalCenterOnFieldWithOffset);
      }
    }

    // the goalkeeper will turn to the given position
    state(goalkeeperTurnToTarget)
    {
      transition
      {
        if((std::abs(goalCenterRelativeWithOffset.translation.angle()) < 10_deg))
          goto goalkeeperWalkToTarget;
      }
      action
      {
        skillRequest = SkillRequest::Builder::observe((theRobotPose * goalCenterRelativeWithOffset).translation);
      }
    }

    // while searching for the ball the goalkeeper should avoid touching the ball to avoid scoring an own goal.
    state(goalkeeperAvoidBall)
    {
      transition
      {
        const float GoalkeeperToGoalLineCenter = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f)).norm();
        if(GoalkeeperToGoalLineCenter > initialRadius + minRadius)
          goto goalkeeperTurnToTarget;
      }
      action
      {
        const Vector2f goalCenterRelative(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f));
        const Vector2f offsetRelative(goalCenterRelative.normalized(-200.f));
        const Vector2f offsetAbsolute(theRobotPose * offsetRelative);
        skillRequest = SkillRequest::Builder::walkTo(offsetAbsolute);
      }
    }
  }
};
#undef action
#undef transition

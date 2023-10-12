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

#include "Representations/BehaviorControl/Skills.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX BallSearch::
#endif
#include "Tools/Cabsl.h"

class BallSearch : public Cabsl<BallSearch>, public BehaviorBase
{
public:
  bool ballUnknown = false;
  const float cornerPositionOffset = 500.f;
  const int refereeBallPlacementDelay = 5000;
  const float refereeBallPlacementAccuracy = 400.f;
  bool teammatesBallModelInCorner = false;
  bool ballModelIsInOneCorner = false;
  const float positionOffset = 10.f;

  BallSearch();

  void preProcess() override;

  void postProcess() override;

  SkillRequest execute(const Agent& agent, const Agents& otherAgents);

  SkillRequest decidePositionForSearch(const Vector2f leftPosition, const Vector2f rightPosition, const Agent& agent, const Agents& otherAgents);

private:
  ActivationGraph activationGraph;

  //Own corner kick possible ball positions
  const std::array<Vector2f, 2> opponentCorners =
  {
    Vector2f(theFieldDimensions.xPosOpponentGroundLine - positionOffset, theFieldDimensions.yPosLeftSideline - positionOffset),
    Vector2f(theFieldDimensions.xPosOpponentGroundLine - positionOffset, theFieldDimensions.yPosRightSideline + positionOffset)
  };
  //Own goal kick possible ball positions
  const std::array<Vector2f, 2> ownGoalCorners =
  {
    Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea),
    Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea)
  };
  //Opponent corner kick possible ball positions
  const std::array<Vector2f, 2> ownCorners =
  {
    Vector2f(theFieldDimensions.xPosOwnGroundLine + positionOffset, theFieldDimensions.yPosLeftSideline - positionOffset),
    Vector2f(theFieldDimensions.xPosOwnGroundLine + positionOffset, theFieldDimensions.yPosRightSideline + positionOffset)
  };
  //Opponent goal kick possible ball positions
  const std::array<Vector2f, 2> opponentGoalCorners =
  {
    Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea),
    Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea)
  };
  // skill request for ballSearch bevaior
  SkillRequest skillRequest;
  Agents agents;
  const float groundLineXOffset = 50.f;
  const float minRadius = 20.f;
  float initialRadius;
  const Agent* agent;

  option(Root)
  {
    const Pose2f goalCenterOnFieldWithOffset = Pose2f(0.f, theFieldDimensions.xPosOwnGroundLine + groundLineXOffset, 0.f);
    const Pose2f goalCenterRelativeWithOffset = theRobotPose.inverse() * goalCenterOnFieldWithOffset;
    //the initial ballSearch is used in non-standard situations
    initial_state(initial)
    {
      transition
      {
        ANNOTATION("BallSearch", "is active");
        if(agent->isGoalkeeper)
          goto goalkeeper;
        else if(theGameState.isCornerKick())
          goto cornerKick;
        else if(theGameState.isGoalKick())
          goto goalKick;
        else
          goto gridSearch;
      }
    }
    // if the ball is lost, the robot will use the ballSearchAreas Grid to search the ball.
    state(gridSearch)
    {
      transition
      {
        if(theGameState.isCornerKick())
        {
          goto cornerKick;
        }
        else if(theGameState.isGoalKick())
        {
          goto goalKick;
        }
        else if(agent->isGoalkeeper)
        {
          goto goalkeeper;
        }
      }
      action
      {
        skillRequest = SkillRequest::Builder::observe(theBallSearchAreas.cellToSearchNext(*agent));
      }
    }
    state(cornerKick)
    {
      transition
      {
        if(!theGameState.isCornerKick())
          goto gridSearch;
      }
      action
      {
        const auto& corners = theGameState.isForOpponentTeam() ? ownCorners : opponentCorners;

        skillRequest = decidePositionForSearch(corners[0], corners[1], *agent, agents);
      }
    }
    state(goalKick)
    {
      transition
      {
        if(!theGameState.isGoalKick())
          goto gridSearch;
      }
      action
      {
        const auto& corners = theGameState.isForOpponentTeam() ? opponentGoalCorners : ownGoalCorners;

        skillRequest = decidePositionForSearch(corners[0], corners[1], *agent, agents);
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
          else if(theFieldBall.timeSinceBallWasSeen > 10000 && theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundLine)
            goto goalkeeperWalkToTarget;
          else if(theFieldBall.timeSinceBallWasSeen > 10000 && (std::abs(goalCenterRelativeWithOffset.translation.angle()) > 45_deg))
          {
            initialRadius = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f)).norm();;
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
    state(goalkeeperWalkToTarget)
    {
      action
      {
        skillRequest = SkillRequest::Builder::walkTo(goalCenterOnFieldWithOffset);
      }
    }
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
    state(goalkeeperAvoidBall)
    {
      transition
      {
        const float GoalkeeperToGoalLineCenter = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f)).norm();
        if(GoalkeeperToGoalLineCenter > initialRadius + minRadius)
          goto goalkeeperTurnToTarget;
      }
      action
      {
        const Vector2f goalCenterRelative(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f));
        const Vector2f offsetRelative(goalCenterRelative.normalized(-200.f));
        const Vector2f offsetAbsolute(theRobotPose * offsetRelative);
        skillRequest = SkillRequest::Builder::walkTo(offsetAbsolute);
      }
    }
  }
};
#undef action
#undef transition

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

  SkillRequest execute(const Agent& agent);

private:
  ActivationGraph activationGraph;
  SkillRequest skillRequest; /**< Skill request for ballSearch behavior */
  static constexpr float goalLineXOffset = 50.f;
  static constexpr float minRadius = 20.f;
  float initialRadius;
  const Agent* agent;

  option(Root)
  {
    const Pose2f goalCenterOnFieldWithOffset = Pose2f(0.f, theFieldDimensions.xPosOwnGoalLine + goalLineXOffset, 0.f);
    const Pose2f goalCenterRelativeWithOffset = theRobotPose.inverse() * goalCenterOnFieldWithOffset;

    //the initial ballSearch
    initial_state(initial)
    {
      transition
      {
        ANNOTATION("BallSearch", "is active");
        if(agent->isGoalkeeper)
          goto goalkeeper;
        else
          goto fieldPlayer;
      }
    }

    // if the ball is lost in a non-standard situation, the robot will use the ball search particles to search the ball.
    state(fieldPlayer)
    {
      transition
      {
        if(agent->isGoalkeeper)
          goto goalkeeper;
      }
      action
      {
        skillRequest = SkillRequest::Builder::observe(theBallSearchParticles.positionToSearch(*agent));
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

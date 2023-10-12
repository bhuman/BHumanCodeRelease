/**
 * @file PlayBall.cpp
 *
 * This file defines an implementation of a skill that plays the ball (under consideration of the skill request).
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(PlayBallImpl,
{,
  IMPLEMENTS(PlayBall),
  CALLS(GoToBallAndDribble),
  CALLS(HandleBallAtOwnGoalPost),
  CALLS(KickAtGoal),
  CALLS(PassToTeammate),
  CALLS(Zweikampf),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(SkillRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) goalPostHandlingAreaRadius,
    (float)(1.2f) goalPostHandlingAreaHysteresisMultiplier,
    (float)(1000.f) duelMinDistanceToClosest,
    (Angle)(100_deg) duelMinAngleToClosest,
  }),
});

class PlayBallImpl : public PlayBallImplBase
{
  option(PlayBall)
  {
    const bool isLeft = theFieldBall.positionOnField.y() > 0.f;
    const Vector2f usedGoalPost(std::min(theFieldDimensions.xPosOwnGoalPost, theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius), isLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);

    initial_state(executeSkillRequest)
    {
      transition
      {
        if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() < sqr(goalPostHandlingAreaRadius))
          goto atOwnGoalPost;
        if(activateDuel())
          goto zweikampf;
      }

      action
      {
        switch(theSkillRequest.skill)
        {
          case SkillRequest::pass:
            thePassToTeammateSkill({.playerNumber = theSkillRequest.passTarget});
            break;
          case SkillRequest::dribble:
            theGoToBallAndDribbleSkill({.targetDirection = Angle::normalize(theSkillRequest.target.rotation - theRobotPose.rotation)});
            break;
          case SkillRequest::shoot:
          default:
            theKickAtGoalSkill();
            break;
        }
      }
    }

    state(atOwnGoalPost)
    {
      transition
      {
        if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() > sqr(goalPostHandlingAreaRadius * goalPostHandlingAreaHysteresisMultiplier))
        {
          if(activateDuel())
            goto zweikampf;
          goto executeSkillRequest;
        }
      }

      action
      {
        theHandleBallAtOwnGoalPostSkill();
      }
    }

    state(zweikampf)
    {
      transition
      {
        if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() < sqr(goalPostHandlingAreaRadius))
          goto atOwnGoalPost;
        if(stopDuel())
          goto executeSkillRequest;
      }

      action
      {
        theZweikampfSkill();
      }
    }
  }

  bool activateDuel() const
  {
    if((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam())
      return false;

    const auto [distanceToClosestObstacle, angleToClosestObstacle, smallestAngleToCloseObstacle] = [&]() -> std::tuple<float, Angle, Angle>
    {
      float x = std::numeric_limits<float>::max();
      Angle y = 0_deg;
      Angle z = 180_deg;
      Vector2f nearestObstacle;
      for(const auto& o : theObstacleModel.obstacles)
      {
        const float d = (o.center - theFieldBall.positionRelative).squaredNorm();
        if(d < x)
        {
          x = d;
          y = o.center.angle();
          if(x < sqr(duelMinDistanceToClosest))
            z = std::min(Angle(std::abs(y)), z);
        }
      }
      return
      {
        std::sqrt(x), y, z
      };
    }();

    const bool obstacleClose = distanceToClosestObstacle < (duelMinDistanceToClosest * (1.f - (std::abs(angleToClosestObstacle) / duelMinAngleToClosest / 3.f)));
    const bool obstacleAngleClose = smallestAngleToCloseObstacle <= duelMinAngleToClosest;
    const bool ballSeen = theFieldBall.ballWasSeen(300);
    const bool ballPosXClose = between<float>(theFieldBall.positionRelative.x(), 0.f, 500.f);
    const bool ballPosYClose = std::abs(theFieldBall.positionRelative.y()) < 300.f;
    const bool rotationToBallOk = std::abs(theFieldBall.positionRelative.angle()) < 80_deg;

    return obstacleClose && obstacleAngleClose &&
           ballSeen && ballPosXClose && ballPosYClose && rotationToBallOk;
  };

  bool stopDuel() const
  {
    if((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam())
      return true;

    if(theFieldBall.positionRelative.x() > 1000.f)
      return true;

    for(const Obstacle& omo : theObstacleModel.obstacles)
    {
      const float distanceToObstacle = (omo.center - theFieldBall.positionRelative).norm();
      const float angleToObstacle = std::abs(omo.center.angle());

      if(distanceToObstacle < (duelMinDistanceToClosest * 1.2f * (1.f - (std::abs(angleToObstacle) / pi / 2.f))))
        return false;
    }
    return true;
  };
};

MAKE_SKILL_IMPLEMENTATION(PlayBallImpl);

/**
 * @file InterceptBall.cpp
 *
 * This file implements an implementation of the InterceptBall skill.
 *
 * @author Arne Hasselbring
 */

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(InterceptBallImpl,
{,
  IMPLEMENTS(InterceptBall),
  CALLS(Annotation),
  CALLS(Dive),
  CALLS(LookAtBall),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(BehaviorParameters),
  REQUIRES(FieldBall),
});

class InterceptBallImpl : public InterceptBallImplBase
{
  option(InterceptBall)
  {
    initial_state(chooseAction)
    {
      transition
      {
        ASSERT(state_time == 0);
        ASSERT(p.interceptionMethods != 0);
        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();

        // TODO all thresholds should scale with the distance to the ball.
        // For example we do not want to jump for a ball 1 mm in front of us, but we also do not want to jump for a ball 4 meters far away.

        unsigned interceptionMethods = p.interceptionMethods;
        left = positionIntersectionYAxis > 0.f;
        if(left)
          interceptionMethods &= ~bit(Interception::jumpRight);
        else
          interceptionMethods &= ~bit(Interception::jumpLeft);
        ASSERT(interceptionMethods != 0);

        if((interceptionMethods & bit(Interception::stand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.standRadius, theBehaviorParameters.standRadius) || interceptionMethods < (bit(Interception::stand) << 1)))
          goto stand;

        if((interceptionMethods & bit(Interception::walk)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.walkRadius, theBehaviorParameters.walkRadius) || interceptionMethods < (bit(Interception::walk) << 1)))
          goto walk;

        if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
        {
          if(p.allowDive)
            goto genuflectStand;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
        {
          if(p.allowDive)
            goto genuflectStandDefender;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        ASSERT(interceptionMethods & bit(Interception::walk));
        goto walk;
      }
    }

    state(stand)
    {
      transition
      {
        if(!(theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto targetStand;

        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();
        if(!between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.standRadius - 40.f, theBehaviorParameters.standRadius + 40.f) || !(p.interceptionMethods & bit(Interception::stand)))
        {
          unsigned interceptionMethods = p.interceptionMethods;
          left = positionIntersectionYAxis > 0.f;
          if(left)
            interceptionMethods &= ~bit(Interception::jumpRight);
          else
            interceptionMethods &= ~bit(Interception::jumpLeft);
          ASSERT(interceptionMethods != 0);

          if((interceptionMethods & bit(Interception::walk)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.walkRadius, theBehaviorParameters.walkRadius)))
            goto walk;

          if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
          {
            if(p.allowDive)
              goto genuflectStand;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
          {
            if(p.allowDive)
              goto genuflectStandDefender;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
          {
            if(p.allowDive)
              goto keeperSitJump;
            else
              goto audioJump;
          }

          if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
          {
            if(p.allowDive)
              goto keeperSitJump;
            else
              goto audioJump;
          }

          if(interceptionMethods & bit(Interception::walk))
            goto walk;
        }

        ASSERT(p.interceptionMethods & bit(Interception::stand));
      }
      action
      {
        theAnnotationSkill({.annotation = "Intercept Ball Stand!"});
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(walk)
    {
      transition
      {
        if(!(theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto targetStand;

        ASSERT(p.interceptionMethods != 0);
        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();

        unsigned interceptionMethods = p.interceptionMethods;
        left = positionIntersectionYAxis > 0.f;
        if(left)
          interceptionMethods &= ~bit(Interception::jumpRight);
        else
          interceptionMethods &= ~bit(Interception::jumpLeft);
        ASSERT(interceptionMethods != 0);

        if((interceptionMethods & bit(Interception::stand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.standRadius, theBehaviorParameters.standRadius) || interceptionMethods < (bit(Interception::stand) << 1)))
          goto stand;

        if(!((interceptionMethods & bit(Interception::walk)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.walkRadius, theBehaviorParameters.walkRadius))))
        {
          if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
          {
            if(p.allowDive)
              goto genuflectStand;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
          {
            if(p.allowDive)
              goto genuflectStandDefender;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
          {
            if(p.allowDive)
              goto keeperSitJump;
            else
              goto audioJump;
          }

          if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -theBehaviorParameters.jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
          {
            if(p.allowDive)
              goto keeperSitJump;
            else
              goto audioJump;
          }

          ASSERT(interceptionMethods & bit(Interception::walk));
        }
      }
      action
      {
        theAnnotationSkill({.annotation = "Intercept Ball Walk!"});
        theLookAtBallSkill();
        theWalkToPointSkill({.target = {0.f, 0.f, theFieldBall.intersectionPositionWithOwnYAxis.y()},
                             .reduceWalkingSpeed = false,
                             .disableAligning = true});
      }
    }

    state(genuflectStand)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto targetStand;
      }
      action
      {
        theAnnotationSkill({.annotation = left ? "Genuflect Left!" : "Genuflect Right!"});
        theLookForwardSkill();
        theDiveSkill({.request = left ? MotionRequest::Dive::squatArmsBackLeft : MotionRequest::Dive::squatArmsBackRight});
      }
    }

    state(genuflectStandDefender)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto targetStand;
      }
      action
      {
        theAnnotationSkill({.annotation = left ? "Genuflect Defender Left!" : "Genuflect Defender Right!"});
        theLookForwardSkill();
        theDiveSkill({.request = left ? MotionRequest::Dive::squatWideArmsBackLeft : MotionRequest::Dive::squatWideArmsBackRight});
      }
    }

    state(keeperSitJump)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000)
          goto targetStand;
      }
      action
      {
        theAnnotationSkill({.annotation = left ? "Keeper Jump Left!" : "Keeper Jump Right!"});
        theLookAtBallSkill();
        theDiveSkill({.request = left ? MotionRequest::Dive::jumpLeft : MotionRequest::Dive::jumpRight});
      }
    }

    state(audioGenuflect)
    {
      transition
      {
        goto targetStand;
      }
      action
      {
        theSaySkill({.text = "Genuflect"});
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(audioJump)
    {
      transition
      {
        goto targetStand;
      }
      action
      {
        theSaySkill({.text = left ? "Jump left" : "Jump right"});
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    target_state(targetStand)
    {
      action
      {
        theLookAtBallSkill();
        theStandSkill();
      }
    }
  }

  bool left; /**< Whether the interception is left of the robot (right otherwise). */
};

MAKE_SKILL_IMPLEMENTATION(InterceptBallImpl);

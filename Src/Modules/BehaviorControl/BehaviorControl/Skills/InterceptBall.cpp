/**
 * @file InterceptBall.cpp
 *
 * This file implements an implementation of the InterceptBall and AfterInterceptBall skills.
 *
 * @author Arne Hasselbring
 */

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(InterceptBallImpl,
{,
  IMPLEMENTS(InterceptBall),
  IMPLEMENTS(AfterInterceptBall),
  CALLS(Annotation),
  CALLS(GetUpEngine),
  CALLS(KeyframeMotion),
  CALLS(LookAtBall),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(MotionInfo),
  USES(MotionRequest),
  DEFINES_PARAMETERS(
  {,
    /* Note: These thresholds also exist in the behavior parameters, but have a slightly other purpose there. */
    (float)(60.f) standRadius, /**< The range that is covered by just standing. */
    (float)(150.f) genuflectRadius, /**< The range that is covered with the penalty keeper genuflect. */
    (float)(230.f) genuflectStandRadius, /**< The range that is covered with a genuflect. */
    (float)(500.f) jumpRadius, /**< The range that is covered with a keeper jump. */
  }),
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

        unsigned interceptionMethods = p.interceptionMethods;
        left = positionIntersectionYAxis > 0.f;
        if(left)
          interceptionMethods &= ~bit(Interception::jumpRight);
        else
          interceptionMethods &= ~bit(Interception::jumpLeft);
        ASSERT(interceptionMethods != 0);

        if((interceptionMethods & bit(Interception::stand)) && (between<float>(positionIntersectionYAxis, -standRadius, standRadius) || interceptionMethods < (bit(Interception::stand) << 1)))
          goto stand;

        if((interceptionMethods & bit(Interception::genuflectFromSitting)) && (between<float>(positionIntersectionYAxis, -genuflectRadius, genuflectRadius) || interceptionMethods < (bit(Interception::genuflectFromSitting) << 1)))
        {
          if(p.allowDive)
            goto genuflectFromSitting;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
        {
          if(p.allowDive)
            goto genuflectStand;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
        {
          if(p.allowDive)
            goto genuflectStandDefender;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
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
        if(!between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -standRadius - 40.f, standRadius + 40.f) || !(p.interceptionMethods & bit(Interception::stand)))
        {
          unsigned interceptionMethods = p.interceptionMethods;
          left = positionIntersectionYAxis > 0.f;
          if(left)
            interceptionMethods &= ~bit(Interception::jumpRight);
          else
            interceptionMethods &= ~bit(Interception::jumpLeft);
          ASSERT(interceptionMethods != 0);

          if((interceptionMethods & bit(Interception::genuflectFromSitting)) && (between<float>(positionIntersectionYAxis, -genuflectRadius, genuflectRadius) || interceptionMethods < (bit(Interception::genuflectFromSitting) << 1)))
          {
            if(p.allowDive)
              goto genuflectFromSitting;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
          {
            if(p.allowDive)
              goto genuflectStand;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
          {
            if(p.allowDive)
              goto genuflectStandDefender;
            else
              goto audioGenuflect;
          }

          if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
          {
            if(p.allowDive)
              goto keeperSitJump;
            else
              goto audioJump;
          }

          if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
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
        theAnnotationSkill("Intercept Ball Stand!");
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

        if((interceptionMethods & bit(Interception::stand)) && (between<float>(positionIntersectionYAxis, -standRadius, standRadius) || interceptionMethods < (bit(Interception::stand) << 1)))
          goto stand;

        if((interceptionMethods & bit(Interception::genuflectFromSitting)) && (between<float>(positionIntersectionYAxis, -genuflectRadius, genuflectRadius) || interceptionMethods < (bit(Interception::genuflectFromSitting) << 1)))
        {
          if(p.allowDive)
            goto genuflectFromSitting;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
        {
          if(p.allowDive)
            goto genuflectStand;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
        {
          if(p.allowDive)
            goto genuflectStandDefender;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        ASSERT(interceptionMethods & bit(Interception::walk));
      }
      action
      {
        theAnnotationSkill("Intercept Ball Walk!");
        theLookAtBallSkill();
        theWalkToPointSkill(Pose2f(0.f, 0.f, theFieldBall.intersectionPositionWithOwnYAxis.y()), 1.f, /* rough: */ false, /* disableObstacleAvoidance: */ false, /* disableAligning: */ true);
      }
    }

    state(genuflectFromSitting)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -180.f, 180.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto getUp;
      }
      action
      {
        theAnnotationSkill("Genuflect!");
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectFromSitting);
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
          goto getUp;
      }
      action
      {
        theAnnotationSkill(left ? "Genuflect Left!" : "Genuflect Right!");
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStand, /* mirror: */ !left);
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
          goto getUp;
      }
      action
      {
        theAnnotationSkill(left ? "Genuflect Defender Left!" : "Genuflect Defender Right!");
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStandDefender, /* mirror: */ !left);
      }
    }

    state(keeperSitJump)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000)
          goto getUp;
      }
      action
      {
        theAnnotationSkill(left ? "Keeper Jump Left!" : "Keeper Jump Right!");
        theLookAtBallSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft, /* mirror: */ !left);
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
        theSaySkill("Genuflect");
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
        theSaySkill(left ? "Jump left" : "Jump right");
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(getUp)
    {
      transition
      {
        if(theGetUpEngineSkill.isDone())
          goto targetStand;
      }
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
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

  option(AfterInterceptBall)
  {
    initial_state(initial)
    {
      transition
      {
        ASSERT(state_time == 0);
        ASSERT(theMotionRequest.motion == MotionRequest::keyframeMotion && theMotionRequest.keyframeMotionRequest.keyframeMotion >= KeyframeMotionRequest::firstNonGetUpAction);

        // It can be assumed that AfterInterceptBall and InterceptBall are not used simultaneously. Therefore, left is reused here.
        left = !theMotionRequest.keyframeMotionRequest.mirror;

        if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectFromSitting)
          goto genuflectFromSitting;
        else if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStand)
          goto genuflectStand;
        else if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStandDefender)
          goto genuflectStandDefender;
        else if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::keeperJumpLeft)
          goto keeperSitJump;

        FAIL("This skill must only be called when a diving motion has been set last.");
      }
    }

    state(genuflectFromSitting)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -180.f, 180.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto getUp;
      }
      action
      {
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectFromSitting, /* mirror: */ left);
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
          goto getUp;
      }
      action
      {
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStand, /* mirror: */ !left);
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
          goto getUp;
      }
      action
      {
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStandDefender, /* mirror: */ !left);
      }
    }

    state(keeperSitJump)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000)
          goto getUp;
      }
      action
      {
        theLookAtBallSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft, /* mirror: */ !left);
      }
    }

    state(getUp)
    {
      transition
      {
        if(theGetUpEngineSkill.isDone())
          goto getUpTarget;
      }
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
      }
    }

    target_state(getUpTarget)
    {
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
      }
    }
  }

  bool left; /**< Whether the interception is left of the robot (right otherwise). */
};

MAKE_SKILL_IMPLEMENTATION(InterceptBallImpl);

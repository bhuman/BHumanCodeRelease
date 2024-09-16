/**
 * @file InterceptBall.cpp
 *
 * This file implements the InterceptBall skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/Interception.h"

option((SkillBehaviorControl) InterceptBall,
       args((unsigned) interceptionMethods,
            (bool) allowGetUp,
            (bool) allowDive),
       vars((bool)(false) left, /**< Whether the interception is left of the robot (right otherwise). */
            (Interception::Method)(Interception::numOfMethods) lastMethod)) /**< Last used interception from previous frame. */
{
  auto replaceJumpWithWalk = [&](const unsigned interceptionMethods) -> bool
  {
    return interceptionMethods & bit(Interception::walk) && // walking is allowed
    theFieldInterceptBall.timeUntilIntersectsOwnYAxis * 1000.f < theBehaviorParameters.timeForJump; // jumping would be too late
  };

  auto getWalkRadius = [&]() -> float
  {
    return mapToRange(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, theBehaviorParameters.timeForIntercetionForMaxWalkRadius.min, theBehaviorParameters.timeForIntercetionForMaxWalkRadius.max, theBehaviorParameters.walkRadius.min, theBehaviorParameters.walkRadius.max);
  };

  auto getIntersectionAction = [&](const Interception::Method currentMethod, const unsigned interceptionMethods)
  {
    const float positionIntersectionYAxis = theFieldInterceptBall.intersectionPositionWithOwnYAxis.y();
    unsigned filteredInterceptionMethods = interceptionMethods;

    left = positionIntersectionYAxis > 0.f;

    if(left)
      filteredInterceptionMethods &= ~bit(Interception::jumpRight);
    else
      filteredInterceptionMethods &= ~bit(Interception::jumpLeft);
    ASSERT(filteredInterceptionMethods != 0);

    const float standHysterese = currentMethod == Interception::stand ? 40.f : 0.f;

    if((filteredInterceptionMethods & bit(Interception::stand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.standRadius - standHysterese, theBehaviorParameters.standRadius + standHysterese) || filteredInterceptionMethods < (bit(Interception::stand) << 1)))
      return Interception::stand;
    if((filteredInterceptionMethods & bit(Interception::walk)) && (between<float>(positionIntersectionYAxis, -getWalkRadius(), getWalkRadius()) || filteredInterceptionMethods < (bit(Interception::walk) << 1)))
      return Interception::walk;
    if((filteredInterceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || filteredInterceptionMethods < (bit(Interception::genuflectStand) << 1)))
      return Interception::genuflectStand;
    if((filteredInterceptionMethods & bit(Interception::genuflectStandDefender)) && (between<float>(positionIntersectionYAxis, -theBehaviorParameters.genuflectStandRadius, theBehaviorParameters.genuflectStandRadius) || filteredInterceptionMethods < (bit(Interception::genuflectStandDefender) << 1)))
      return Interception::genuflectStandDefender;
    if(!replaceJumpWithWalk(filteredInterceptionMethods) && (filteredInterceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < theBehaviorParameters.jumpRadius || filteredInterceptionMethods < (bit(Interception::jumpLeft) << 1)))
      return Interception::jumpLeft;
    if(!replaceJumpWithWalk(filteredInterceptionMethods) && (filteredInterceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -theBehaviorParameters.jumpRadius || filteredInterceptionMethods < (bit(Interception::jumpRight) << 1)))
      return Interception::jumpRight;

    ASSERT(filteredInterceptionMethods & bit(Interception::walk));
    return Interception::walk;
  };

  common_transition
  {
    // TODO figure out if this is needed
    /* if(!theFieldInterceptBall.interceptBall)
      goto targetStand; */
    // We can always switch if the keeper motion did not start yet or none was requested
    if(!(theMotionInfo.isMotion(MotionPhase::keyframeMotion) &&
         lastMethod == Interception::genuflectStand &&
         lastMethod == Interception::jumpLeft && lastMethod == Interception::jumpRight))
    {
      switch(getIntersectionAction(lastMethod, interceptionMethods))
      {
        case Interception::stand:
        {
          if(lastMethod != Interception::stand)
            goto stand;
          break;
        }
        case Interception::walk:
        {
          if(lastMethod != Interception::walk)
            goto walk;
          break;
        }
        case Interception::genuflectStand:
        {
          if(lastMethod == Interception::genuflectStand)
            break;
          if(allowDive)
            goto genuflectStand;
          else
            goto audioGenuflect;
        }
        case  Interception::genuflectStandDefender:
        {
          if(lastMethod == Interception::genuflectStandDefender)
            break;
          if(allowDive)
            goto genuflectStandDefender;
          else
            goto audioGenuflect;
        }
        case Interception::jumpLeft:
        case Interception::jumpRight:
        {
          if(lastMethod == Interception::jumpLeft || lastMethod == Interception::jumpRight)
            break;
          if(allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }
      }
    }
  }

  initial_state(chooseAction)
  {
    transition
    {
      ASSERT(state_time == 0);
      ASSERT(interceptionMethods != 0);
    }
  }

  state(stand)
  {
    transition
    {
      if(!(theFieldBall.ballWasSeen(300) &&
           between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
        goto targetStand;

      ASSERT(interceptionMethods & bit(Interception::stand));
    }
    action
    {
      lastMethod = Interception::stand;
      Annotation({.annotation = "Intercept Ball Stand!"});
      LookAtBall();
      Stand();
    }
  }

  state(walk)
  {
    transition
    {
      if(!(theFieldBall.ballWasSeen(300) &&
           between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
        goto targetStand;

      ASSERT(interceptionMethods != 0);
      ASSERT(interceptionMethods & bit(Interception::walk));
    }
    action
    {
      lastMethod = Interception::walk;
      Annotation({.annotation = "Intercept Ball Walk!"});
      LookAtBall();
      WalkToPoint({.target = {0.f, 0.f, theFieldInterceptBall.intersectionPositionWithOwnYAxis.y()},
                   .reduceWalkingSpeed = ReduceWalkSpeedType::noChange,
                   .disableAligning = true});
    }
  }

  state(genuflectStand)
  {
    transition
    {
      if(allowGetUp && state_time > 2000 &&
         !(between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
           theFieldBall.ballWasSeen(300) &&
           between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
        goto targetStand;
    }
    action
    {
      lastMethod = Interception::genuflectStand;
      Annotation({.annotation = left ? "Genuflect Left!" : "Genuflect Right!"});
      LookForward();
      Dive({.request = left ? MotionRequest::Dive::squatArmsBackLeft : MotionRequest::Dive::squatArmsBackRight});
    }
  }

  state(genuflectStandDefender)
  {
    transition
    {
      if(allowGetUp && state_time > 2000 &&
         !(between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
           theFieldBall.ballWasSeen(300) &&
           between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
        goto targetStand;
    }
    action
    {
      lastMethod = Interception::genuflectStandDefender;
      Annotation({.annotation = left ? "Genuflect Defender Left!" : "Genuflect Defender Right!"});
      LookForward();
      Dive({.request = left ? MotionRequest::Dive::squatWideArmsBackLeft : MotionRequest::Dive::squatWideArmsBackRight});
    }
  }

  state(keeperSitJump)
  {
    transition
    {
      if(allowGetUp && state_time > 2000)
        goto targetStand;
    }
    action
    {
      lastMethod = left ? Interception::jumpLeft : Interception::jumpRight;
      Annotation({.annotation = left ? "Keeper Jump Left!" : "Keeper Jump Right!"});
      LookAtBall();
      Dive({.request = left ? MotionRequest::Dive::jumpLeft : MotionRequest::Dive::jumpRight});
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
      lastMethod = Interception::genuflectStand;
      Say({.text = "Genuflect"});
      LookAtBall();
      Stand();
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
      lastMethod = left ? Interception::jumpLeft : Interception::jumpRight;
      Say({.text = left ? "Jump left" : "Jump right"});
      LookAtBall();
      Stand();
    }
  }

  target_state(targetStand)
  {
    action
    {
      lastMethod = Interception::stand;
      LookAtBall();
      Stand();
    }
  }
}

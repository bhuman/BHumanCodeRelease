/**
 * @file FreeBallHoldingEngine.cpp
 *
 * This file implements a module that
 *
 * @author Harm Thordsen
 */

#include "Debugging/DebugDrawings.h"
#include "Debugging/Annotation.h"
#include "FreeBallHoldingEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(FreeBallHoldingEngine);

void FreeBallHoldingEngine::update(FreeBallHoldingGenerator& theFreeBallHoldingGenerator)
{
  if(theMotionRequest.motion != MotionRequest::freeBallHolding)
    state = FreeBallState::none;
  theFreeBallHoldingGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position);

    Pose2f stepSize;
    switch(state)
    {
      case FreeBallState::none:
        state = FreeBallState::align;
      case FreeBallState::align:
        if(std::abs(motionRequest.targetDirection) > 70_deg)
        {
          stepSize = Pose2f(motionRequest.targetDirection, 0.f, 0.f);
          break;
        }
        state = FreeBallState::rotate;
      case FreeBallState::rotate:
        stepSize = Pose2f(isLeftPhase ? 90_deg : -90_deg, 0.f, 0.f);
        state = FreeBallState::sideStep;
        break;
      case FreeBallState::sideStep:
        stepSize = Pose2f(0.f, 0.f, isLeftPhase ? 1000.f : -1000.f);
        state = FreeBallState::zeroStep;
        break;
      case FreeBallState::zeroStep:
        stepSize = Pose2f();
        state = FreeBallState::align;
        break;
      default:
        FAIL("Invalid FreeBallState");
        break;
    }
    stepSize.rotation = theWalkGenerator.getRotationRange(isLeftPhase, motionRequest.walkSpeed).clamped(stepSize.rotation);

    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    theWalkGenerator.getTranslationPolygon(isLeftPhase, stepSize.rotation, lastPhase, motionRequest.walkSpeed, translationPolygon, translationPolygonNoCenter, false, false);

    if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), stepSize.translation))
    {
      Vector2f intersectionPoint;
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f), stepSize.translation), intersectionPoint, false));
      stepSize.translation = intersectionPoint;
    }

    if(isLeftPhase == (stepSize.translation.y() < 0.f) && stepSize.translation.y() != 0.f)
    {
      stepSize.translation.y() = 0.f;
      FAIL("FreeBallHoldingEngine sideTranslation set to 0");
    }

    return theWalkGenerator.createPhase(stepSize, lastPhase, 0.f);
  };
}

#include "Tools/Motion/WalkUtilities.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Framework/Settings.h"
#include "Libs/MathBase/BHMath.h"
#include "Tools/Motion/MotionPhase.h"

namespace WalkUtilities
{
  void calcSideWalk(const WalkGenerator& theWalkGenerator, const WalkingEngineOutput& walkOutput,
                    const WalkStepData&, const Pose2f& walkSpeedRatio,
                    const Pose2f& targetInSCS, Pose2f& modTargetInSCS,
                    const Vector2f& targetOfInterest, const bool isLeftPhase, const bool isFastWalk,
                    const MotionPhase& lastPhase, const bool useModTarget)
  {
    const Rangea maxRange(-Constants::pi, Constants::pi);

    const Angle angleFocus = targetOfInterest.angle();
    const Rangea angleFocusRange(maxTargetFocusAngle.min + angleFocus, maxTargetFocusAngle.max + angleFocus);

    const Angle walkDirection = useModTarget ? modTargetInSCS.translation.angle() : targetInSCS.translation.angle();

    // Determine which 90 degree angle relative to the walk direction is closer to the target of interest.
    const Angle leftSide = walkDirection + 90_deg;
    const Angle rightSide = walkDirection - 90_deg;
    const Angle useSideWalkAngle = std::abs(Angle::normalize(angleFocus - leftSide)) < 90_deg ? Angle::normalize(leftSide) : Angle::normalize(rightSide);

    // Based on the walk translation polygon get the two direction angles to the side, which allow for max side speed
    std::vector<Vector2f> translationPolygon;
    const Vector2f frontLeft = walkOutput.maxStepSize.translation;
    const Vector2f backRight = Vector2f(-7.f, -walkOutput.maxStepSize.translation.y());
    theWalkGenerator.generateTranslationPolygon(isLeftPhase, 0, walkSpeedRatio, translationPolygon, backRight, frontLeft, true, false);
    ASSERT(translationPolygon.size() > 1);

    Vector2f frontPoint = translationPolygon[0];
    frontPoint.x() *= 0.5f;
    const Angle forwardEdgeAngle = frontPoint.angle();
    Angle backwardEdgeAngle = translationPolygon[translationPolygon.size() - 1].angle();
    // The original first and last polygon point were the same and therefore filtered out.
    // The code assumes translationPolygon[translationPolygon.size() - 1].x() being <= 0 and translationPolygon[translationPolygon.size() - 1].y() >= 0.
    // Also the code assumes at max side speed some backward translation (x-axis) is still allowed.
    // At least one of those assumptions are violated now. Therefore take the first point instead.
    if(translationPolygon[translationPolygon.size() - 1].y() != frontPoint.y())
      backwardEdgeAngle = forwardEdgeAngle;

    Rangea sideWalkRange;
    if(walkDirection < 0)
      sideWalkRange = Rangea(-(backwardEdgeAngle - 90_deg), (90_deg - forwardEdgeAngle));
    else
      sideWalkRange = Rangea(-(90_deg - forwardEdgeAngle), (backwardEdgeAngle - 90_deg));

    Rangea mirrorAngleFocusRange, mirrorAllowedOrientation;
    // Make sure the robot turns to the correct direction
    // Special handling needed if the angles are outside the -pi and +pi range
    bool mirrorFocus = false;
    if(!maxRange.isInside(angleFocusRange.min))
    {
      mirrorFocus = true;
      mirrorAngleFocusRange = Rangea(angleFocusRange.min + Constants::pi2, angleFocusRange.max + Constants::pi2);
    }
    else if(!maxRange.isInside(angleFocusRange.max))
    {
      mirrorFocus = true;
      mirrorAngleFocusRange = Rangea(angleFocusRange.min - Constants::pi2, angleFocusRange.max - Constants::pi2);
    }

    Rangea const* useFocusRange = &angleFocusRange;
    if(mirrorFocus)
    {
      const Angle diffFocus1 = std::abs(angleFocus - angleFocusRange.limit(angleFocus));
      const Angle diffFocus2 = std::abs(angleFocus - mirrorAngleFocusRange.limit(angleFocus));
      useFocusRange = diffFocus1 <= diffFocus2 ? &angleFocusRange : &mirrorAngleFocusRange;
    }

    // Finally determine the rotation
    modTargetInSCS.rotation = (useSideWalkAngle) - sideWalkRange.limit(useSideWalkAngle);
    if(!useFocusRange->isInside(modTargetInSCS.rotation))  // if not inside, clip it into the vision range with some buffer range
      modTargetInSCS.rotation = Rangea(useFocusRange->min + extraFocusShiftIfBallNotInVision, useFocusRange->max - extraFocusShiftIfBallNotInVision).limit(modTargetInSCS.rotation);

    // Optimize rotation to allow max translation steps
    if(isLeftPhase != (modTargetInSCS.translation.y() < 0.f) && std::abs(modTargetInSCS.translation.y()) > 20.f)
    {
      const Rangea nextStepRotationRange = theWalkGenerator.getRotationRange(!isLeftPhase, walkSpeedRatio, modTargetInSCS); // The next step can only do this much rotation. Only this mount can be reduced and optimized during the current step
      const Angle nextStepRotation = nextStepRotationRange.limit(modTargetInSCS.rotation);
      const Angle currentStepMinRotation = modTargetInSCS.rotation - nextStepRotation; // This rotation part must be executed
      const Angle maxStepRotation = theWalkGenerator.getStepRotationRange(isLeftPhase, walkSpeedRatio, modTargetInSCS.translation, isFastWalk, lastPhase, true, false).limit(modTargetInSCS.rotation);

      modTargetInSCS.rotation = modTargetInSCS.rotation >= 0.f ? std::max(maxStepRotation, currentStepMinRotation) : std::min(maxStepRotation, currentStepMinRotation);
    }

    if(targetInSCS.translation.squaredNorm() < sqr(300.f))
    {
      const Angle originalRotation = modTargetInSCS.rotation;
      const float interpolation = Rangef::ZeroOneRange().limit(std::max(0.f, std::abs((targetOfInterest.rotated(-targetInSCS.rotation)).angle()) - 45_deg) / 20_deg);
      modTargetInSCS.rotation = originalRotation * interpolation + (1.f - interpolation) * Rangea(std::min(0_deg, targetInSCS.rotation), std::max(0_deg, targetInSCS.rotation)).limit(modTargetInSCS.rotation);
    }
  }

  void calcDiagonal(const WalkStepData& walkStepData, Pose2f& modTargetInSCS, const Vector2f& targetOfInterest, const bool isFastWalk)
  {
    const float correctionFactorOffset = isFastWalk ? 1000.f : 0.f;
    const float rotationCorrectionFactor = 1.f + Rangef::ZeroOneRange().limit((walkStepData.stepTarget.translation.x() - correctionFactorOffset) / 1000.f);

    const Angle walkAngle = modTargetInSCS.translation.angle();
    const Angle angleFocus = targetOfInterest.angle();
    const Rangea angleFocusRange(WalkUtilities::maxTargetFocusAngle.min + angleFocus, WalkUtilities::maxTargetFocusAngle.max + angleFocus);

    Rangea mirrorAngleFocusRange = angleFocusRange;
    const Rangea maxRange(-Constants::pi, Constants::pi);
    if(!maxRange.isInside(angleFocusRange.min))
      mirrorAngleFocusRange = Rangea(angleFocusRange.min + Constants::pi2, angleFocusRange.max + Constants::pi2);
    else if(!maxRange.isInside(angleFocusRange.max))
      mirrorAngleFocusRange = Rangea(angleFocusRange.min - Constants::pi2, angleFocusRange.max - Constants::pi2);

    const Angle diff1 = std::abs(angleFocus - angleFocusRange.limit(angleFocus));
    const Angle diff2 = std::abs(angleFocus - mirrorAngleFocusRange.limit(angleFocus));
    const Rangea& useFocusRange = diff1 <= diff2 ? angleFocusRange : mirrorAngleFocusRange;

    Angle clippedWalkAngle = useFocusRange.limit(walkAngle);
    if(useFocusRange.isInside(walkAngle) && modTargetInSCS.translation != Vector2f::Zero())
    {
      const Angle rotateWalkTarget = clippedWalkAngle * (1.f - Rangef::ZeroOneRange().limit((std::abs(clippedWalkAngle - Rangea(-5_deg, 5_deg).limit(clippedWalkAngle)) * 2.f)));
      const float maxTranslationXPreview = 300.f * std::min(1.f, std::max(0.f, (modTargetInSCS.translation.norm() - 500.f)) / 1500.f);
      clippedWalkAngle = useFocusRange.limit((modTargetInSCS.translation - Vector2f(std::min(modTargetInSCS.translation.x() * 0.9f, maxTranslationXPreview), 0.f)).angle()) * rotationCorrectionFactor;
      modTargetInSCS.translation.rotate(-rotateWalkTarget);
    }
    else
      clippedWalkAngle *= rotationCorrectionFactor;
    modTargetInSCS.rotation = clippedWalkAngle;
  }
}

#include "Tools/Motion/WalkUtilities.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Motion/MotionPhase.h"

namespace WalkUtilities
{
  void calcSideWalk(const WalkGenerator& theWalkGenerator, const WalkingEngineOutput& walkOutput, const Pose2f walkSpeedRatio,
                    const Pose2f& targetInSCS, Pose2f& modTargetInSCS,
                    const Vector2f& targetOfInterest, const bool isLeftPhase, const bool isFastWalk,
                    const MotionPhase& lastPhase, const bool useModTarget)
  {
    const Rangea maxRange(-Constants::pi, Constants::pi);

    const Angle angleFocus = targetOfInterest.angle();
    const Rangea angleFocusRange(maxTargetFocusAngle.min + angleFocus, maxTargetFocusAngle.max + angleFocus);

    Angle walkDirection = useModTarget ? modTargetInSCS.translation.angle() : targetInSCS.translation.angle();

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
    const Angle backwardEdgeAngle = translationPolygon[translationPolygon.size() - 1].angle();

    Rangea sideWalkRange;
    Angle sideOffset;
    if(walkDirection > 0)
    {
      sideWalkRange = Rangea(useSideWalkAngle - (backwardEdgeAngle - 90_deg), useSideWalkAngle + (90_deg - forwardEdgeAngle));
      sideOffset = -90_deg;
    }
    else
    {
      sideWalkRange = Rangea(useSideWalkAngle - (90_deg - forwardEdgeAngle), useSideWalkAngle + (backwardEdgeAngle - 90_deg));
      sideOffset = 90_deg;
    }

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
      const Angle targetAngleAtModTranslation = (targetOfInterest - modTargetInSCS.translation).angle();
      const Angle diffFocus1 = std::abs(targetAngleAtModTranslation - angleFocusRange.limit(targetAngleAtModTranslation));
      const Angle diffFocus2 = std::abs(targetAngleAtModTranslation - mirrorAngleFocusRange.limit(targetAngleAtModTranslation));
      useFocusRange = diffFocus1 < diffFocus2 ? &angleFocusRange : &mirrorAngleFocusRange;
    }

    // Finally determine the rotation
    modTargetInSCS.rotation = useFocusRange->limit(sideWalkRange.limit(walkDirection + sideOffset));

    // Optimize rotation to allow max translation steps
    if(isLeftPhase != (modTargetInSCS.translation.y() < 0.f) && std::abs(modTargetInSCS.translation.y()) > 20.f)
      modTargetInSCS.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, walkSpeedRatio, modTargetInSCS.translation, isFastWalk, lastPhase, true).limit(modTargetInSCS.rotation);
  }

  void calcDiagonal(const WalkGenerator& theWalkGenerator, const WalkingEngineOutput& walkOutput, const Pose2f walkSpeedRatio, const Pose2f& targetInSCS,
                    Pose2f& modTargetInSCS, const Vector2f& targetOfInterest, const bool isLeftPhase,
                    const bool isFastWalk, const MotionPhase& lastPhase)
  {
    const Rangea maxRange(-Constants::pi, Constants::pi);

    const Angle angleFocus = targetOfInterest.angle();
    const Rangea angleFocusRange(WalkUtilities::maxTargetFocusAngle.min + angleFocus, WalkUtilities::maxTargetFocusAngle.max + angleFocus);
    Angle walkDirection = targetInSCS.translation.angle();

    // Based on the walk translation polygon get the two direction angles to the front, which allow for the max diagonal speed
    // two angles -> left and right diagonal. Clip walkDirection into this range at the end
    std::vector<Vector2f> translationPolygon;
    const Vector2f frontLeft = walkOutput.maxStepSize.translation;
    const Vector2f backRight = Vector2f(-walkOutput.maxBackwardStepSize, -walkOutput.maxStepSize.translation.y());

    theWalkGenerator.generateTranslationPolygon(isLeftPhase, 0, walkSpeedRatio, translationPolygon, backRight, frontLeft, true, false);
    ASSERT(translationPolygon.size() > 1);
    std::size_t polygonIndex = 0;

    int numberOfPositivePoints = 0;
    for(const Vector2f& p : translationPolygon)
      if(p.x() > 0)
        numberOfPositivePoints++;
    if(numberOfPositivePoints == 4)
      polygonIndex = 1;

    ASSERT(numberOfPositivePoints > 0);
    const Angle walkDiagonalAngle = translationPolygon[polygonIndex].angle() / 3.f; // At the GORE 2023 we used 2.f. The diagonal steps were a bit too big.
    const Rangea optimalWalkAngleRange(-walkDiagonalAngle, walkDiagonalAngle);

    walkDirection = walkDirection - optimalWalkAngleRange.limit(walkDirection);
    const Angle targetAngleAtModTranslation = (targetOfInterest - modTargetInSCS.translation).angle();
    Rangea mirrorAngleFocusRange = angleFocusRange;
    if(!maxRange.isInside(angleFocusRange.min))
      mirrorAngleFocusRange = Rangea(angleFocusRange.min + Constants::pi2, angleFocusRange.max + Constants::pi2);
    else if(!maxRange.isInside(angleFocusRange.max))
      mirrorAngleFocusRange = Rangea(angleFocusRange.min - Constants::pi2, angleFocusRange.max - Constants::pi2);

    const Angle diff1 = std::abs(targetAngleAtModTranslation - angleFocusRange.limit(targetAngleAtModTranslation));
    const Angle diff2 = std::abs(targetAngleAtModTranslation - mirrorAngleFocusRange.limit(targetAngleAtModTranslation));
    modTargetInSCS.rotation = diff1 < diff2 ? Angle::normalize(angleFocusRange.limit(walkDirection)) : Angle::normalize(mirrorAngleFocusRange.limit(walkDirection));

    // Optimize walk path to prevent walking on a circle path
    if(std::abs(modTargetInSCS.translation.y()) > 200.f && modTargetInSCS.translation.x() > 200.f && std::abs(modTargetInSCS.rotation - walkDirection) < 5_deg)
    {
      const Angle modAngle = modTargetInSCS.translation.angle();
      modTargetInSCS.translation.rotate(((modAngle > 0 ? 1.f : -1.f) * 2.f * walkDiagonalAngle) - modAngle);
    }

    // Optimize rotation to allow max translation steps
    if(isLeftPhase != (modTargetInSCS.translation.y() < 0.f) && std::abs(modTargetInSCS.translation.y()) > 20.f)
      modTargetInSCS.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, walkSpeedRatio, modTargetInSCS.translation, isFastWalk, lastPhase, true).limit(modTargetInSCS.rotation);
  }
}

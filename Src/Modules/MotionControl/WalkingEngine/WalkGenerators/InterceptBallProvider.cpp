/**
 * @file InterceptBallProvider.cpp
 *
 * This file implements a module that intercepts the ball
 *
 * @author Florian Scholz
 * @author Philip Reichenberg
 */

#include "InterceptBallProvider.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Framework/Settings.h"
#include "Debugging/Annotation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/Transformation.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Libs/Math/Approx.h"

MAKE_MODULE(InterceptBallProvider);

using namespace Motion::Transformation;

void InterceptBallProvider::update(InterceptBallGenerator& theInterceptBallGenerator)
{
  DEBUG_DRAWING("motion:RobotPose", "drawingOnField") // Set the origin to the robot's current position
  {
    ORIGIN("motion:RobotPose", theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation);
  }

  DECLARE_DEBUG_DRAWING("module:InterceptBallProvider:hitPos", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:InterceptBallProvider:line", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:InterceptBallProvider:rect", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("module:InterceptBallProvider");

  if(!calculatedRectangles)
  {
    leftFootBackRight = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius, theWalkStepData.yHipOffset - footRectYOffset);
    leftFootFrontLeft = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset, theWalkStepData.yHipOffset + footRectYOffset);
    rightFootBackRight = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius, -theWalkStepData.yHipOffset - footRectYOffset);
    rightFootFrontLeft = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset, -theWalkStepData.yHipOffset + footRectYOffset);

    leftFootBackRightSmall = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectXSmallOffset, theWalkStepData.yHipOffset - footRectYSmallOffset);
    leftFootFrontLeftSmall = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset - footRectXSmallOffset, theWalkStepData.yHipOffset + footRectYSmallOffset);
    rightFootBackRightSmall = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectXSmallOffset, -theWalkStepData.yHipOffset - footRectYSmallOffset);
    rightFootFrontLeftSmall = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset - footRectXSmallOffset, -theWalkStepData.yHipOffset + footRectYSmallOffset);
    leftFootRectSmall = Geometry::Rect(leftFootBackRightSmall, leftFootFrontLeftSmall);
    rightFootRectSmall = Geometry::Rect(rightFootBackRightSmall, rightFootFrontLeftSmall);

    backRightIntercepting = Vector2f(0.f, -theWalkStepData.yHipOffset);
    frontLeftIntercepting = Vector2f(bestHitPoint, theWalkStepData.yHipOffset);

    calculatedRectangles = true;
  }

  theInterceptBallGenerator.intercept = [this](const MotionRequest& motionRequest, const std::vector<Vector2f>& translationPolygon, const bool isLeftPhase, const std::optional<Vector2f>& oldStep)
  {
    return intercept(motionRequest, 0, isLeftPhase, translationPolygon, oldStep);
  };

  theInterceptBallGenerator.kickIntercept = [this](const MotionRequest& motionRequest, const std::vector<Vector2f>& translationPolygon, const bool isLeftPhase, const Pose2f& oldStep)
  {
    // Kick we can not execute -> intercept ball
    KickInfo::KickType kickType = motionRequest.motion == MotionRequest::dribble ? getDribbleKick(motionRequest, isLeftPhase) : motionRequest.kickType;
    if(!checkKickType(kickType) || !(motionRequest.motion == MotionRequest::walkToBallAndKick || motionRequest.motion == MotionRequest::dribble))
      return oldStep;

    // Determine which ball velocity shall be used
    Vector2f ballVelocity = motionRequest.ballEstimate.velocity;
    if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
    {
      Vector2f ballDirection = theFieldInterceptBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
      if(ballDirection.squaredNorm() == 0.f)
      {
        ballDirection = theFieldInterceptBall.intersectionPositionWithOwnYAxis - motionRequest.ballEstimate.position;
        ASSERT(ballDirection.squaredNorm() != 0.f); // Once x Intersection is actually used, this needs to be adjusted
      }

      if(ballDirection.squaredNorm() == 0.f)
        return oldStep;

      ballVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
    }

    // Apply scsCognition transformation
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);
    const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation));

    WalkKickVariant walkKickVariant;
    Pose2f kickPose;
    float timeForDistanceHitPoint;
    // Only the kickPose is valid afterwards. DO NOT use walkKickVariant
    static_cast<void>(planCurrentKick(walkKickVariant, kickPose, timeForDistanceHitPoint, kickType, ballLine, scsCognition, motionRequest, isLeftPhase, 0, translationPolygon));
    return kickPose;
  };

  theInterceptBallGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    // Kick we can not execute -> intercept ball
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position);
    KickInfo::KickType kickType = motionRequest.motion == MotionRequest::dribble ? getDribbleKick(motionRequest, isLeftPhase) : motionRequest.kickType;
    if(!checkKickType(kickType) || !(motionRequest.motion == MotionRequest::walkToBallAndKick || motionRequest.motion == MotionRequest::dribble))
      return theWalkGenerator.createPhase(intercept(motionRequest, &lastPhase, theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position), std::vector<Vector2f>(), std::optional<Vector2f>()), lastPhase, 0.f);

    const bool kickIsLeftPhase = theKickInfo[kickType].kickLeg == Legs::left;

    // Determine which ball velocity shall be used
    Vector2f ballVelocity = motionRequest.ballEstimate.velocity;
    if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
    {
      Vector2f ballDirection = theFieldInterceptBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
      if(ballDirection.squaredNorm() == 0.f)
      {
        ballDirection = theFieldInterceptBall.intersectionPositionWithOwnYAxis - motionRequest.ballEstimate.position;
        ASSERT(ballDirection.squaredNorm() != 0.f); // Once x Intersection is actually used, this needs to be adjusted
      }

      if(ballDirection.squaredNorm() == 0.f)
        return theWalkGenerator.createPhase(Pose2f(0.f, 0.001f, 0.f), lastPhase, 0.f);

      ballVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
    }

    const float baseTimeToKickThreshold = lastPhase.type == MotionPhase::stand ? timeToKickThresholdStand : timeToKickThresholdWalk;

    // Apply scsCognition transformation
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, kickIsLeftPhase);
    const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation));

    WalkKickVariant walkKickVariant;
    Pose2f kickPose;
    float timeForDistanceHitPoint;
    const bool canExecuteCurrentKick = planCurrentKick(walkKickVariant, kickPose, timeForDistanceHitPoint, kickType, ballLine, scsCognition, motionRequest, kickIsLeftPhase, &lastPhase, std::vector<Vector2f>());
    if(isLeftPhase == kickIsLeftPhase && (canExecuteCurrentKick || timeForDistanceHitPoint < baseTimeToKickThreshold + timeOffset))
    {
      auto nextPhase = theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, false);
      if(nextPhase)
      {
        ANNOTATION("module:InterceptBallProvider", "Intercept Ball with Kick");
        return nextPhase;
      }
    }

    const KickInfo::KickType mirrorType = theKickInfo.mirror(kickType);
    const bool otherKickIsLeftPhase = theKickInfo[mirrorType].kickLeg == Legs::left;

    WalkKickVariant mirrorWalkKickVariant;
    Pose2f mirrorKickPose;
    float mirrorTimeForDistanceHitPoint;

    const Pose2f scsCognitionOther = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, otherKickIsLeftPhase);
    const Geometry::Line ballLineOther = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation));
    const bool canExecuteMirrorKick = planCurrentKick(mirrorWalkKickVariant, mirrorKickPose, mirrorTimeForDistanceHitPoint, mirrorType, ballLineOther, scsCognitionOther, motionRequest, otherKickIsLeftPhase, &lastPhase, std::vector<Vector2f>());
    if(isLeftPhase == otherKickIsLeftPhase && (canExecuteMirrorKick || mirrorTimeForDistanceHitPoint < baseTimeToKickThreshold + timeOffset + timeToKickThresholdMirrorBonus))
    {
      auto nextPhase = theWalkKickGenerator.createPhase(mirrorWalkKickVariant, lastPhase, false);
      if(nextPhase)
      {
        ANNOTATION("module:InterceptBallProvider", "Intercept Ball with Kick");
        return nextPhase;
      }
    }

    // Both legs can not really kick right now. Check if the current step is our last chance to kick the ball before it is too late
    // It is better to try to kick, than not even trying, even if it looks stupid
    // The planned kick leg is next and the time is below the threshold
    // The mirrored leg afterwards has even less time, or the ball is rolling too fast as waiting another step could also be too late
    // TODO based on the distance to the hit point, scale the time thresholds. If the ball is too far away, it might be better to risk another step to get closer
    if(isLeftPhase == kickIsLeftPhase && walkKickVariant.ballEstimationTime > 0.f && timeForDistanceHitPoint < baseTimeToKickThreshold + timeOffset && // either now or never, not much time left anymore
       (mirrorTimeForDistanceHitPoint < timeForDistanceHitPoint || mirrorTimeForDistanceHitPoint < timeOffset + baseTimeToKickThreshold + timeToKickThresholdMirrorBonus)) // the other leg would be inside the time threshold
    {
      auto nextPhase = theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, false);
      if(nextPhase)
      {
        ANNOTATION("module:InterceptBallProvider", "Intercept Ball with Kick");
        return nextPhase;
      }
    }

    // Check the same for the mirrored kick
    if(isLeftPhase == otherKickIsLeftPhase && mirrorTimeForDistanceHitPoint < baseTimeToKickThreshold + timeOffset + timeToKickThresholdMirrorBonus &&
       (timeForDistanceHitPoint < mirrorTimeForDistanceHitPoint || timeForDistanceHitPoint < timeOffset + baseTimeToKickThreshold))
    {
      auto nextPhase = theWalkKickGenerator.createPhase(mirrorWalkKickVariant, lastPhase, false);
      if(nextPhase)
      {
        ANNOTATION("module:InterceptBallProvider", "Intercept Ball with Kick");
        return nextPhase;
      }
    }

    // No kick was executed. Update kickPose with the real next swing phase
    if(isLeftPhase != kickIsLeftPhase)
    {
      const Pose2f scsCognitionReal = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);
      const Geometry::Line ballLineReal = Geometry::Line(scsCognitionReal * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognitionReal.rotation));

      WalkKickVariant walkKickVariantReal;
      Pose2f kickPoseReal;
      float timeForDistanceHitPointReal;
      static_cast<void>(planCurrentKick(walkKickVariantReal, kickPoseReal, timeForDistanceHitPointReal, kickType, ballLineReal, scsCognitionReal, motionRequest, isLeftPhase, &lastPhase, std::vector<Vector2f>()));

      kickPose = kickPoseReal;
      timeForDistanceHitPoint = timeForDistanceHitPointReal;
    }

    // Make sure the sole closer to the ball starts first
    if(lastPhase.type == MotionPhase::stand)
    {
      if(isLeftPhase)
        kickPose.translation.y() = std::max(kickPose.translation.y(), 0.01f);
      else
        kickPose.translation.y() = std::min(kickPose.translation.y(), -0.01f);
    }

    ASSERT(!std::isnan(kickPose.translation.x()));
    ASSERT(!std::isnan(kickPose.translation.y()));
    ASSERT(!std::isnan(kickPose.rotation));

    if(lastPhase.type == MotionPhase::stand && std::abs(kickPose.rotation) < 5_deg && std::abs(kickPose.translation.x()) < mapToRange(timeForDistanceHitPoint, 0.75f, 1.f, keepStandingThreshold.min, keepStandingThreshold.max))
      return theWalkGenerator.createPhase(Pose2f(), lastPhase, 0.f);
    return theWalkGenerator.createPhase(kickPose, lastPhase, 0.f);
  };
}

bool InterceptBallProvider::planCurrentKick(WalkKickVariant& walkKickVariant, Pose2f& kickPose, float& timeForDistanceHitPoint,
                                            const KickInfo::KickType& kickType, const Geometry::Line& ballLine, const Pose2f& scsCognition,
                                            const MotionRequest& motionRequest, const bool isLeftPhase, const MotionPhase* lastPhase,
                                            const std::vector<Vector2f>& translationPolygon)
{
  Vector2f orthPoint;
  optimizeKick(kickType, kickPose, orthPoint, ballLine, scsCognition, motionRequest);
  if(!Approx::isEqual((orthPoint - ballLine.base).angle(), ballLine.direction.angle(), static_cast<float>(1_deg)))
  {
    kickPose = Pose2f();
    return false;
  }

  const Angle plannedRotation = kickPose.rotation;

  // calc ttrp before clipping
  if(lastPhase)
    clipPose(kickPose, isLeftPhase, *lastPhase, true, false);
  else
    clipPose(kickPose, isLeftPhase, translationPolygon);

  // Optimize x translation
  timeForDistanceHitPoint = BallPhysics::timeForDistance(ballLine.direction, (orthPoint - ballLine.base).norm(), theBallSpecification.friction);
  if(Geometry::isPointInsideRectangle(!isLeftPhase ? rightFootRectSmall : leftFootRectSmall, orthPoint))
  {
    const float ratio = mapToRange(timeForDistanceHitPoint, kickPoseSizeScaling.min, kickPoseSizeScaling.max, 0.f, 1.f);
    kickPose.translation.x() *= ratio;
  }

  // Get as much rotation, without reducing the translation
  if(lastPhase)
    kickPose.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, *lastPhase, true, true).limit(kickPose.rotation);
  else
  {
    kickPose.rotation = theWalkGenerator.getStepRotationRangeOther(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, translationPolygon, true, true).limit(kickPose.rotation);
    return false; // WalkKickVariant stuff is not necessary afterwards. We only care for the kickPose
  }

  // Set up walk kick
  walkKickVariant = WalkKickVariant(kickType, theKickInfo[kickType].walkKickType, theKickInfo[kickType].kickLeg, KickPrecision::justHitTheBall, motionRequest.kickLength, motionRequest.targetDirection, false);
  walkKickVariant.ballEstimationTime = timeForDistanceHitPoint;
  DelayKickParams delayParams;
  delayParams.delay = std::max(timeForDistanceHitPoint - timeOffset, 0.01f);
  delayParams.kickIndex = 1;
  walkKickVariant.delayParams = delayParams;
  walkKickVariant.ignoreNoStandPhaseRule = true;
  walkKickVariant.forceSwingFoot = isLeftPhase;

  // TODO check if delay is even possible

  // only used to modify the walkKicKVariant (mainly direction), return is ignored intentionally
  theWalkKickGenerator.canStart(walkKickVariant, *lastPhase, motionRequest.directionPrecision, PreStepType::notAllowed, false);

  // Drawings
  DEBUG_RESPONSE("module:InterceptBallProvider")
  {
    const Vector2f& backRight = !isLeftPhase ? rightFootBackRightSmall : leftFootBackRightSmall;
    const Vector2f& frontLeft = !isLeftPhase ? rightFootFrontLeftSmall : leftFootFrontLeftSmall;
    const Vector2f backRightInRobot = scsCognition.inverse() * backRight;
    const Vector2f frontLeftInRobot = scsCognition.inverse() * frontLeft;
    RECTANGLE("module:InterceptBallProvider:rect", backRightInRobot.x(), backRightInRobot.y(), frontLeftInRobot.x(), frontLeftInRobot.y(), 15, Drawings::solidPen, ColorRGBA::black);

    const Vector2f drawableOrth = scsCognition.inverse() * orthPoint;
    CROSS("module:InterceptBallProvider:hitPos", drawableOrth.x(), drawableOrth.y(), 100, 20, Drawings::solidPen, ColorRGBA::magenta);
    LINE("module:InterceptBallProvider:line", motionRequest.ballEstimate.position.x(), motionRequest.ballEstimate.position.y(), motionRequest.ballEstimate.position.x() + ballLine.direction.x(), motionRequest.ballEstimate.position.y() + ballLine.direction.y(), 20, Drawings::solidPen, ColorRGBA::red);
  }

  // if we can kick (position and timing wise)
  const Vector2f& backRightKick = !isLeftPhase ? rightFootBackRight : leftFootBackRight;
  const Vector2f& frontLeftKick = !isLeftPhase ? rightFootFrontLeft : leftFootFrontLeft;
  const bool ballLineIntersectKickArea = Geometry::isPointInsideRectangle(backRightKick, frontLeftKick, orthPoint);
  const float baseTimeToKickThreshold = lastPhase->type == MotionPhase::stand ? timeToKickThresholdStand : timeToKickThresholdWalk;
  if(ballLineIntersectKickArea && timeForDistanceHitPoint - timeOffset < baseTimeToKickThreshold &&
     std::abs(plannedRotation) < rotationThreshold && isLeftPhase == (theKickInfo[kickType].kickLeg == Legs::left))
  {
    // TODO Wait time for kick needs a function to update the wait time each frame
    // For long wait times the ball perception will always be off
    // This will cause the kick to be bad as a result. This can be fixed by updating the wait time each frame
    auto nextPhase = theWalkKickGenerator.createPhase(walkKickVariant, *lastPhase, false);
    if(nextPhase)
    {
      ANNOTATION("module:InterceptBallProvider", "Intercept Ball with Kick");
      //return nextPhase;
      return true;
    }
  }
  return false;
}

void InterceptBallProvider::optimizeKick(const KickInfo::KickType& kickType, Pose2f& kickPose, Vector2f& orthPoint,
                                         const Geometry::Line& ballLine, const Pose2f& scsCognition,
                                         const MotionRequest& motionRequest)
{
  // Get best point to hit the ball, which is used for the kick pose and the walk kick
  orthPoint = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, -(theKickInfo[kickType].ballOffset.rotated(scsCognition.rotation + motionRequest.targetDirection + theKickInfo[kickType].rotationOffset)));
  // TODO compare with time left and zero step

  Vector2f p1, p2;
  const float hipOffset = -theKickInfo[kickType].ballOffset.y();
  // We want to optimize for as less walk step size as possible
  const bool interpolateKickPose = Geometry::getIntersectionOfLines(ballLine, Geometry::Line(Vector2f(0.f, hipOffset), Vector2f(1.f, 0.f)), p1);

  // Calculate kickPose
  kickPose = Pose2f(scsCognition.rotation + motionRequest.targetDirection, orthPoint);
  kickPose.rotate(theKickInfo[kickType].rotationOffset);
  kickPose.rotation = Angle::normalize(kickPose.rotation);
  kickPose.translate(theKickInfo[kickType].ballOffset);

  if(interpolateKickPose)
  {
    Pose2f kickPoseOther = Pose2f(scsCognition.rotation + motionRequest.targetDirection, p1);
    kickPoseOther.rotate(theKickInfo[kickType].rotationOffset);
    // correct rotation to allow for as less movement as possible
    kickPoseOther.rotation -= motionRequest.directionPrecision.limit(kickPoseOther.rotation);
    kickPoseOther.rotation = Angle::normalize(kickPoseOther.rotation);
    kickPoseOther.translate(theKickInfo[kickType].ballOffset);

    const Vector2f closestPoint = Geometry::getOrthogonalProjectionOfPointOnLine(Geometry::Line(kickPose.translation, kickPoseOther.translation - kickPose.translation), Vector2f::Zero());
    const Vector2f& minPose = kickPose.translation.x() < kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    const Vector2f& maxPose = kickPose.translation.x() >= kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    if(minPose.x() != maxPose.x())
    {
      const float interpolationFactor = mapToRange(closestPoint.x(), minPose.x(), maxPose.x(), 0.f, 1.f);
      kickPose.translation = (1.f - interpolationFactor) * minPose + interpolationFactor * maxPose;
      orthPoint = (1.f - interpolationFactor) * orthPoint + interpolationFactor * p1;
    }
  }

  kickPose.translation.y() = Rangef(-10.f, 10.f).limit(kickPose.translation.y());
}

Pose2f InterceptBallProvider::intercept(const MotionRequest& motionRequest, const MotionPhase* lastPhase, const bool isLeftPhase,
                                        const std::vector<Vector2f>& translationPolygon, const std::optional<Vector2f>& oldStep)
{
  // Determine which velocity shall be used
  Vector2f useBallVelocity = motionRequest.ballEstimate.velocity;
  if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
  {
    Vector2f ballDirection = theFieldInterceptBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
    if(ballDirection.squaredNorm() == 0.f)
    {
      ballDirection = theFieldInterceptBall.intersectionPositionWithOwnYAxis - motionRequest.ballEstimate.position;
      ASSERT(ballDirection.squaredNorm() != 0.f); // Once x Intersection is actually used, this needs to be adjusted
    }

    // Something is broken, walk in place until behavior functions normally again
    if(ballDirection.squaredNorm() == 0.f)
    {
      return Pose2f(0.f, 0.001f, 0.f);
    }

    useBallVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
  }

  // Get transformation matrix from robot-pose-relative coordinates to start-of-next-walkstep-relative coordinates
  const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);

  // Determine orth point
  const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, useBallVelocity.rotated(scsCognition.rotation));
  Vector2f orthPoint = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, Vector2f(bestHitPoint, 0.f));
  bool interpolateKickPose = false;
  bool noKickPose = false;
  Vector2f p1, p2, p3;
  const bool isRightIntercept = ballLine.base.y() < 0.f == Legs::right;
  if(Approx::isEqual((orthPoint - ballLine.base).angle(), ballLine.direction.angle(), static_cast<float>(1_deg)))
  {
    if(Geometry::getIntersectionPointsOfLineAndRectangle(backRightIntercepting, frontLeftIntercepting, ballLine, p1, p2) &&
       p1.x() != p2.x())
    {
      const float interpolationFactor = mapToRange(0.f, p1.x(), p2.x(), 0.f, 1.f);
      p3 = p2 * interpolationFactor + p1 * (1.f - interpolationFactor);
      interpolateKickPose = true;
      noKickPose = true;
    }
    else
    {
      const float hipOffset = isRightIntercept ? -theWalkStepData.yHipOffset : theWalkStepData.yHipOffset;
      // We want to optimize for as less walk step size as possible
      interpolateKickPose = Geometry::getIntersectionOfLines(ballLine, Geometry::Line(Vector2f(0.f, hipOffset), Vector2f(1.f, 0.f)), p3);
    }
  }
  else
    orthPoint = ballLine.base; // Ball rolled already passed us. There is no correct orth point
  // Calculate kickPose
  Pose2f kickPose = orthPoint;
  kickPose.translate(Vector2f(-bestHitPoint, 0.f));
  if(interpolateKickPose)
  {
    Pose2f kickPoseOther = p3;
    if(!noKickPose)
      kickPoseOther.translate(Vector2f(-bestHitPoint, 0.f));

    const Vector2f closestPoint = Geometry::getOrthogonalProjectionOfPointOnLine(Geometry::Line(kickPose.translation, kickPoseOther.translation - kickPose.translation), Vector2f::Zero());
    const Vector2f& minPose = kickPose.translation.x() < kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    const Vector2f& maxPose = kickPose.translation.x() >= kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    const Vector2f& minOrth = kickPose.translation.x() < kickPoseOther.translation.x() ? orthPoint : p3;
    const Vector2f& maxOrth = kickPose.translation.x() >= kickPoseOther.translation.x() ? orthPoint : p3;
    if(minPose.x() != maxPose.x())
    {
      const float interpolationFactor = mapToRange(closestPoint.x(), minPose.x(), maxPose.x(), 0.f, 1.f);
      kickPose.translation = (1.f - interpolationFactor) * minPose + interpolationFactor * maxPose;
      orthPoint = (1.f - interpolationFactor) * minOrth + interpolationFactor * maxOrth;
    }
  }
  // Get time, ball roll angle and size of shifted contact width
  float timeForDistance = BallPhysics::timeForDistance(useBallVelocity, (orthPoint - ballLine.base).norm(), theBallSpecification.friction);
  if(std::numeric_limits<float>::max() == timeForDistance)
    timeForDistance = BallPhysics::computeTimeUntilBallStops(useBallVelocity, theBallSpecification.friction);

  // Ball is close. Reduce side step size so the ball is just touched with the tip of one of the feet.
  // This prevent the ball to roll through between both legs
  // Also apply this clip before clipPose(). Otherwise the step size is reduced by more than it is intended
  if(timeForDistance < 1.f)
  {
    const Angle correctionAngle = Angle::normalize(ballLine.direction.angle() + 180_deg); // 360deg is one circle, half is 180deg. The direction is just mirrored
    const float yShiftClipped = maxYShift.limit(-Vector2f(theRobotDimensions.footLength, 0.f).rotate(correctionAngle).y());
    const float sideScaling = mapToRange(timeForDistance, 0.5f, 1.f, 1.f, 0.f);
    const Rangef yRange(sideScaling * (-theWalkStepData.yHipOffset + footTipYShift) + std::max(yShiftClipped, 0.f), sideScaling * (theWalkStepData.yHipOffset - footTipYShift) + std::min(yShiftClipped, 0.f));
    const float correctionValue = yRange.limit(kickPose.translation.y());
    if(correctionValue * kickPose.translation.y() > 0.f) // ball is rollin
      kickPose.translation.y() = kickPose.translation.y() - yRange.limit(kickPose.translation.y());
  }
  // previous reduction is intended to be applied
  if(timeForDistance < preventSideKickTime)
  {
    // Allow feet moving together, but not outwards
    Rangef clipSideRange(std::min(scsCognition.translation.y(), 0.f), std::max(scsCognition.translation.y(), 0.f));
    if(oldStep) // we are updating the current step. Allow for old previous value
    {
      clipSideRange.min = std::min(clipSideRange.min, oldStep.value().y());
      clipSideRange.max = std::max(clipSideRange.max, oldStep.value().y());
    }
    if((isLeftPhase && kickPose.translation.y() > 0.f) || (!isLeftPhase && kickPose.translation.y() < 0.f))
      kickPose.translation.y() = clipSideRange.limit(kickPose.translation.y());
  }
  if(lastPhase)
    clipPose(kickPose, isLeftPhase, *lastPhase, false, true);
  else
    clipPose(kickPose, isLeftPhase, translationPolygon);

  // Ball is soon intercepted. A bit more optimized handling
  if(timeForDistance < 1.f)
  {
    // We are close to the target. Start backwalking to not kick the ball away
    if(orthPoint.x() - bestHitPoint < 50.f)  // TODO not sure if this even triggers often enough. So no parameter for the 50!
      kickPose.translation.x() = maxXStep.min;
    else
      // Walk to second best pose
      kickPose.translation.x() = maxXStep.limit(kickPose.translation.x());
  }
  // else: We still got time to stand optimal

  // Ball is rolling at a steep angle -> rotate to allow for better sidewalking
  if(std::abs(ballLine.direction.angle()) < maxBallRollAngle)
    kickPose.rotation = theWalkGenerator.getRotationRange(isLeftPhase, Pose2f(1.f, 1.f, 1.f)).clamped(ballLine.direction.angle() < 0.f ? maxRotationStep : -maxRotationStep);

  // Clip rotation
  if(lastPhase)
    kickPose.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, *lastPhase, true, true).limit(kickPose.rotation);
  else
    kickPose.rotation = theWalkGenerator.getStepRotationRangeOther(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, translationPolygon, true, true).limit(kickPose.rotation);

  // wrong foot -> can't execute side step
  if(Global::getSettings().robotType == Settings::nao && isLeftPhase == (kickPose.translation.y() < 0.f))
    kickPose.translation.y() = 0.f;

  ASSERT(!std::isnan(kickPose.translation.x()));
  ASSERT(!std::isnan(kickPose.translation.y()));
  ASSERT(!std::isnan(kickPose.rotation));
  return kickPose;
}

KickInfo::KickType InterceptBallProvider::getDribbleKick(const MotionRequest& motionRequest, const bool isLeftPhase)
{
  const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);
  return (scsCognition * motionRequest.ballEstimate.position).y() > 0.f ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
}

void InterceptBallProvider::clipPose(Pose2f& pose, bool isLeftPhase, const std::vector<Vector2f>& translationPolygon)
{
  // We use the first edge, so we now the actual max side speed
  // Note that the polygon is already filtered, which means that the last point, which should normally have the same y-value as the first point
  // could be filtered out and now might have a smaller y-value or even be negative.
  const Vector2f& firstEdge = translationPolygon[0];

  // Optimize for side step size
  if(std::abs(pose.translation.y()) >= std::abs(firstEdge.y()))
  {
    const float sign = static_cast<float>(sgnPos(pose.translation.y()));
    pose.translation.y() = firstEdge.y() * sign;
    if(pose.translation.x() >= 0.f)
      pose.translation.x() = std::min(firstEdge.x(), pose.translation.x());
    else
    {
      ASSERT(translationPolygon.size() >= 4);
      const Vector2f& lastEdge = translationPolygon[translationPolygon.size() - 1];
      pose.translation.x() = std::max(lastEdge.x(), pose.translation.x());
    }
  }
  // Make sure step it inside polygon
  if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), pose.translation))
  {
    ASSERT(std::abs(pose.translation.y()) <= std::abs(firstEdge.y()));
    Vector2f intersectionPoint;
    // 0.99f := coordinate is not allowed to be on the polygon edge
    VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, pose.translation.y() * 0.99f), Vector2f(pose.translation.x(), 0.f)), intersectionPoint, false));
    pose.translation = intersectionPoint;
  }

  if(isLeftPhase == (pose.translation.y() < 0.f))
    pose.translation.y() = 0.f;
}

void InterceptBallProvider::clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const bool fastWalk, const bool maxPossibleStepSize)
{
  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> translationPolygonNoCenter;
  // 0 Rotation, because we want full translation. Just ensure the rotation is later clipped
  theWalkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, Pose2f(1.f, 1.f, 1.f), translationPolygon, translationPolygonNoCenter, fastWalk, maxPossibleStepSize);

  clipPose(pose, isLeftPhase, translationPolygon);
}

bool InterceptBallProvider::checkKickType(KickInfo::KickType kickType)
{
  if(!kickBall)
    return false;

  // Kicking a rolling ball is not working good enough
  return kickType == KickInfo::walkForwardsLeftLong ||
         kickType == KickInfo::walkForwardsRightLong ||
         kickType == KickInfo::walkForwardsLeftAlternative ||
         kickType == KickInfo::walkForwardsRightAlternative;
}

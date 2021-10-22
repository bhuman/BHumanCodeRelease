/**
 * @file WalkToBallAndKickEngine.cpp
 *
 * This file implements a module that provides a walk to ball and kick generator.
 *
 * @author Arne Hasselbring
 */

#include "WalkToBallAndKickEngine.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Platform/BHAssert.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/InverseKinematic.h"

MAKE_MODULE(WalkToBallAndKickEngine, motionControl);

void WalkToBallAndKickEngine::update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:forwardTurnInterpolation");
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:UseMirroredForwardKick");
  MODIFY("module:WalkToBallAndKickGenerator", overrideKickPower);
  walkToBallAndKickGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    lastPhaseWasKick = lastPhase.type == MotionPhase::kick;

    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false /* TODO */, lastPhase);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryData.inverse() * motionRequest.odometryData;

    const float timeToReachBall = std::max(1.f, std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));
    const Vector2f ballPosition = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity, timeToReachBall, theBallSpecification.friction);
    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const Angle targetDirection = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);
    Rangea precisionRange = motionRequest.directionPrecision;

    Pose2f kickPose = Pose2f(targetDirection, ballPosition);

    // interpolate between forward and turn kick
    if(motionRequest.turnKickAllowed && motionRequest.preStepAllowed && (motionRequest.kickType == KickInfo::walkForwardsLeft || motionRequest.kickType == KickInfo::walkForwardsRight))
    {
      KickInfo::KickType turnKick = motionRequest.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
      const float factor = Rangef::ZeroOneRange().limit(targetDirection / -theKickInfo[turnKick].rotationOffset);
      kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset * (1.f - factor) + theKickInfo[turnKick].rotationOffset * factor);
      kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset * (1.f - factor) + theKickInfo[turnKick].ballOffset * factor);
    }
    else
    {
      kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset);
      kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset);
    }

    const float kickPoseShiftY = shiftInWalkKickInYDirection(kickPose, motionRequest.kickType, targetDirection);
    kickPose.translation.y() += kickPoseShiftY;

    bool isInPositionForKick = false;
    bool mirrorKick = false;
    const Angle directionThreshold = motionRequest.alignPrecisely ? 2_deg : 3_deg;
    switch(motionRequest.kickType)
    {
      case KickInfo::forwardFastLeft:
      case KickInfo::forwardFastLeftLong:
        isInPositionForKick = ballPosition.y() > forwardFastYThreshold.min && ballPosition.y() < forwardFastYThreshold.max && ballPosition.x() < (motionRequest.kickType == KickInfo::forwardFastLeftLong ? forwardFastLongXMaxThreshold : forwardFastXThreshold.max) && ballPosition.x() > forwardFastXThreshold.min;
        isInPositionForKick &= std::abs(targetDirection + theKickInfo[motionRequest.kickType].rotationOffset) < directionThreshold;
        break;
      case KickInfo::forwardFastRight:
      case KickInfo::forwardFastRightLong:
        isInPositionForKick = -ballPosition.y() > forwardFastYThreshold.min && -ballPosition.y() < forwardFastYThreshold.max && ballPosition.x() < (motionRequest.kickType == KickInfo::forwardFastRightLong ? forwardFastLongXMaxThreshold : forwardFastXThreshold.max) && ballPosition.x() > forwardFastXThreshold.min;
        isInPositionForKick &= std::abs(targetDirection + theKickInfo[motionRequest.kickType].rotationOffset) < directionThreshold;
        break;
      case KickInfo::walkForwardsLeft:
      case KickInfo::walkForwardsLeftLong:
      case KickInfo::walkForwardsRight:
      case KickInfo::walkForwardsRightLong:
      case KickInfo::walkSidewardsRightFootToRight:
      case KickInfo::walkTurnRightFootToLeft:
      case KickInfo::walkSidewardsLeftFootToLeft:
      case KickInfo::walkTurnLeftFootToRight:
      case KickInfo::walkForwardStealBallLeft:
      case KickInfo::walkForwardStealBallRight:
      case KickInfo::walkForwardsRightAlternative:
      case KickInfo::walkForwardsLeftAlternative:
      {
        isInPositionForKick = theWalkKickGenerator.canStart(WalkKickVariant(motionRequest.kickType, theKickInfo[motionRequest.kickType].walkKickType, theKickInfo[motionRequest.kickType].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, motionRequest.targetDirection),
                                                            lastPhase, precisionRange, motionRequest.alignPrecisely, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, kickPoseShiftY);
        DEBUG_RESPONSE("module:WalkToBallAndKickEngine:UseMirroredForwardKick")
          if(!isInPositionForKick)
          {
            if(motionRequest.kickType == KickInfo::walkForwardsLeft)
              isInPositionForKick = theWalkKickGenerator.canStart(WalkKickVariant(KickInfo::walkForwardsRight, theKickInfo[KickInfo::walkForwardsRight].walkKickType, theKickInfo[KickInfo::walkForwardsRight].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, motionRequest.targetDirection),
                                                                  lastPhase, precisionRange, motionRequest.alignPrecisely, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, kickPoseShiftY);
            else if(motionRequest.kickType == KickInfo::walkForwardsRight)
              isInPositionForKick = theWalkKickGenerator.canStart(WalkKickVariant(KickInfo::walkForwardsRight, theKickInfo[KickInfo::walkForwardsLeft].walkKickType, theKickInfo[KickInfo::walkForwardsLeft].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, motionRequest.targetDirection),
                                                                  lastPhase, precisionRange, motionRequest.alignPrecisely, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, kickPoseShiftY);
            mirrorKick = isInPositionForKick;
          }
        break;
      }
      case KickInfo::newKick:
      {
        isInPositionForKick = true;
        break;
      }
      default:
        FAIL("Unknown kick type.");
    }
    isInPositionForKick &= lastPhase.type == MotionPhase::stand || lastPhase.type == MotionPhase::walk; // prevent kicking from potential unstable phases
    isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
    // velocity.norm() does not need to be transformed to another coordinate system.
    if(!(motionRequest.kickType == KickInfo::walkForwardStealBallLeft || motionRequest.kickType == KickInfo::walkForwardStealBallRight)) // we are so close to the ball. The velocity might be a false perception
      isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < maxBallVelocity;
    // isInPositionForKick &= theFrameInfo.getTimeSince() < 3000;
    // TODO: check if has been seen (at least since last kick)
    if(isInPositionForKick && motionRequest.alignPrecisely)
    {
      isInPositionForKick &= true; // TODO: velocity
      // TODO: (lastPerception - estimate.position).norm() < 10
    }

    if(isInPositionForKick)
    {
      auto kickPhase = createKickPhase(motionRequest, lastPhase, motionRequest.targetDirection, mirrorKick, precisionRange, kickPoseShiftY);
      if(kickPhase != nullptr)
      {
        lastPhaseWasKickPossible = isInPositionForKick;
        kickPhase->kickType = motionRequest.kickType;
        return kickPhase;
      }
      else
        OUTPUT_ERROR("Creating Kick Phase of Type " << TypeRegistry::getEnumName(motionRequest.kickType) << " in WalkToBallAndKickEngine returned an empty kick!");
    }

    // force "duck" feet for the forwardSteal kick
    if(motionRequest.kickType == KickInfo::walkForwardStealBallLeft || motionRequest.kickType == KickInfo::walkForwardStealBallRight)   // otherwise get normal kickpose
    {
      const Pose3f supportInTorso3DLeft = theTorsoMatrix * theRobotModel.soleLeft;
      const Pose2f supportInTorsoLeft(supportInTorso3DLeft.rotation.getZAngle(), supportInTorso3DLeft.translation.head<2>());
      const Pose2f scsCognitionLeft = supportInTorsoLeft.inverse() * theOdometryData.inverse() * theMotionRequest.odometryData;
      const Vector2f ballLeft = scsCognitionLeft * theMotionRequest.ballEstimate.position;
      const Pose3f supportInTorso3DRight = theTorsoMatrix * theRobotModel.soleRight;
      const Pose2f supportInTorsoRight(supportInTorso3DRight.rotation.getZAngle(), supportInTorso3DRight.translation.head<2>());
      const Pose2f scsCognitionRight = supportInTorsoRight.inverse() * theOdometryData.inverse() * theMotionRequest.odometryData;
      const Vector2f ballRight = scsCognitionRight * theMotionRequest.ballEstimate.position;
      if(theMotionRequest.ballEstimate.position.x() > 0.f &&
         std::abs(kickPose.translation.y()) < kickPoseMaxYTranslation &&
         kickPose.translation.x() < forwardStealStepPlanningThreshold.translation.x() &&
         ballLeft.y() < -forwardStealStepPlanningThreshold.translation.y() &&
         ballRight.y() > forwardStealStepPlanningThreshold.translation.y() &&
         std::abs(kickPose.rotation + (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft).rotation.getZAngle()) < forwardStealStepPlanningThreshold.rotation)
      {
        kickPose = theWalkKickGenerator.getVShapeWalkStep(isLeftPhase, motionRequest.targetDirection + theKickInfo[motionRequest.kickType].rotationOffset);
        if(std::abs(kickPose.translation.y()) < kickPoseMaxYTranslation)
          return theWalkGenerator.createPhase(kickPose, lastPhase);
      }
    }

    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    if(lastPhaseWasKickPossible && !lastPhaseWasKick && std::abs(kickPose.rotation) < kickPoseThresholds.rotation && std::abs(kickPose.translation.x()) < kickPoseThresholds.translation.x() && std::abs(kickPose.translation.y()) < kickPoseThresholds.translation.y())
    {
      const Vector2f offsetToBall = ballPosition + theKickInfo[motionRequest.kickType].ballOffset;
      kickPose.translation.x() = (std::abs(kickPose.translation.x()) > std::abs(offsetToBall.x()) ? kickPose.translation.x() : offsetToBall.x()) * 1.5f;
      kickPose.translation.y() = (std::abs(kickPose.translation.y()) > std::abs(offsetToBall.y()) ? kickPose.translation.y() : offsetToBall.y()) * 1.5f;
    }
    lastPhaseWasKickPossible = isInPositionForKick;
    return theWalkToBallGenerator.createPhase(kickPose, ballPosition, perceivedBallPosition.norm(), obstacleAvoidanceInSCS, motionRequest.walkSpeed, lastPhase);
  };
}

std::unique_ptr<MotionPhase> WalkToBallAndKickEngine::createKickPhase(const MotionRequest& motionRequest, const MotionPhase& lastPhase, const Angle targetDirection, const bool mirrorKick, const Rangea& precisionRange, const float kickPoseShiftY)
{
  if(theKickInfo[motionRequest.kickType].motion == MotionPhase::walk)
  {
    if(mirrorKick)
    {
      if(motionRequest.kickType == KickInfo::walkForwardsLeft)
        return theWalkKickGenerator.createPhase(WalkKickVariant(KickInfo::walkForwardsRight, theKickInfo[KickInfo::walkForwardsRight].walkKickType, theKickInfo[KickInfo::walkForwardsRight].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, targetDirection), lastPhase, precisionRange, true, kickPoseShiftY);
      else if(motionRequest.kickType == KickInfo::walkForwardsRight)
        return theWalkKickGenerator.createPhase(WalkKickVariant(KickInfo::walkForwardsLeft, theKickInfo[KickInfo::walkForwardsLeft].walkKickType, theKickInfo[KickInfo::walkForwardsLeft].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, targetDirection), lastPhase, precisionRange, true, kickPoseShiftY);
    }
    return theWalkKickGenerator.createPhase(WalkKickVariant(motionRequest.kickType, theKickInfo[motionRequest.kickType].walkKickType, theKickInfo[motionRequest.kickType].kickLeg, overrideKickPower >= 0.f ? overrideKickPower : motionRequest.kickPower, targetDirection), lastPhase, precisionRange, true, kickPoseShiftY);
  }
  else if(theKickInfo[motionRequest.kickType].motion == MotionPhase::kick)
  {
    KickRequest kr;
    kr.kickMotionType = theKickInfo[motionRequest.kickType].kickMotionType;
    kr.mirror = theKickInfo[motionRequest.kickType].mirror;
    kr.armsBackFix = false;
    kr.calcDynPoints = [this, kick = theKickInfo[motionRequest.kickType], power = motionRequest.kickPower](const int phaseNumber) -> std::vector<DynPoint>
    {
      const float usePower = overrideKickPower >= 0.f ? overrideKickPower : power;
      const bool isLeftPhase = kick.kickLeg != Legs::right;
      Vector2f ball = lastStableBall;
      if(phaseNumber <= 2) // after the robot shifts all his weight on the support foot, the torsoMatrix might be off and results in a miss position of the ball by up to 3cm
      {
        const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
        const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
        const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
        const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryData.inverse() * theMotionRequest.odometryData;

        ball = scsCognition * theMotionRequest.ballEstimate.position;
        lastStableBall = ball;
      }

      std::vector<DynPoint> d;
      switch(kick.kickMotionType)
      {
        case KickRequest::kickForwardFast:
        {
          if(ball.y() > 0.f)
            ball.y() *= -1.f;

          // RoboCup 2021 shift. Should be based on internal sensor data
          //ball.y() -= 5.f + Rangef(0.f, 10.f).limit((ball.x() - 180.f) / 2.f); //this is a hit correction factor
          ball.y() -= 5.f; // this is a hit correction factor

          //clipping
          forwardFastYClipRange.clamp(ball.y());

          const float hitBeforeTheoreticalMaxVelocity = 5.f;
          const float requiredDistanceFromStrikeOutToKick = 50.f + usePower * (175.f - 50.f);
          const float optimalHitPointX = std::max(0.f, std::min(ball.x() - theBallSpecification.radius - theRobotDimensions.footLength + hitBeforeTheoreticalMaxVelocity, 50.f));

          const Vector2f kickFootX(optimalHitPointX - 0.5f * requiredDistanceFromStrikeOutToKick, optimalHitPointX + 0.5f * requiredDistanceFromStrikeOutToKick);
          const Vector2f kickFootZ(-190.f + usePower * 20.f, -190.f + usePower * 10.f);

          const Vector3f strikeOut(kickFootX.x(), ball.y(), kickFootZ.x()), kickTo(kickFootX.y(), ball.y(), kickFootZ.y());
          const DynPoint dynRFoot2(Phase::rightFootTra, 2, strikeOut), dynRFoot3(Phase::rightFootTra, 3, kickTo);

          d.push_back(dynRFoot2);
          d.push_back(dynRFoot3);
          break;
        }
        case KickRequest::kickForwardFastLong:
        {
          if(ball.y() > 0.f)
            ball.y() *= -1.f;

          // RoboCup 2021 shift. Should be based on internal sensor data
          //ball.y() -= Rangef(0.f, 40.f).limit((ball.x() - 200.f)); //this is a hit correction factor
          ball.y() -= 5.f; // this is a hit correction factor

          //clipping
          forwardFastYClipRange.clamp(ball.y());

          const Vector2f kickFootX(-70.f, 70.f);
          const Vector2f kickFootZ(-190.f + usePower * 20.f, -190.f + usePower * 10.f);

          const Vector3f strikeOut(kickFootX.x(), ball.y(), kickFootZ.x()), kickTo(kickFootX.y(), ball.y(), kickFootZ.y());
          const DynPoint dynRFoot2(Phase::rightFootTra, 2, strikeOut), dynRFoot3(Phase::rightFootTra, 3, kickTo);

          d.push_back(dynRFoot2);
          d.push_back(dynRFoot3);
          break;
        }
      }
      return d;
    };
    return theKickGenerator.createPhase(kr, lastPhase);
  }
  FAIL("Unknown motion type.");
  return std::unique_ptr<MotionPhase>();
}

float WalkToBallAndKickEngine::shiftInWalkKickInYDirection(const Pose2f& kickPose, const KickInfo::KickType kickType, const Angle direction)
{
  if(theKickInfo[kickType].walkKickType != WalkKicks::forward)
    return 0.f;
  bool leftBlocked = false;
  bool rightBlocked = false;
  for(Obstacle o : theObstacleModel.obstacles)
  {
    const Vector2f oLeftInKickPose = kickPose.inverse() * o.left;
    const Vector2f oRightInKickPose = kickPose.inverse() * o.right;
    leftBlocked |= o.center.y() < 0.f && oLeftInKickPose.squaredNorm() < sqr(350.f) ;
    rightBlocked |= o.center.y() > 0.f && oRightInKickPose.squaredNorm() < sqr(350.f);
  }
  if(!leftBlocked && rightBlocked && kickType == KickInfo::walkForwardsLeft)
    return Rangef::ZeroOneRange().limit(-direction / 45_deg) * (theKickInfo[kickType].ballOffset.y() - theKickInfo[KickInfo::walkTurnLeftFootToRight].ballOffset.y());
  if(leftBlocked && !rightBlocked && kickType == KickInfo::walkForwardsRight)
    return Rangef::ZeroOneRange().limit(direction / 45_deg) * (theKickInfo[kickType].ballOffset.y() - theKickInfo[KickInfo::walkTurnRightFootToLeft].ballOffset.y());
  return 0.f;
}

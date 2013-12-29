/*
 * @file IndykickEngine.cpp
 * This file implements a module that creates kicking motions.
 * @author Felix Wenk
 */

#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/InverseKinematic.h"
#include "Tools/BodyDynamics/InverseDynamics.h"
#include "Tools/Math/Matrix.h"
#include "IndykickEngine.h"
#include <algorithm>

MAKE_MODULE(IndykickEngine, Motion Control)

void IndykickEngine::init()
{
  // Initialize parameters.
  approximateFootContour();

  futureBodyDynamicsProviderDelegate.init();

  const int kickDuration = 750;
  const float kickSpeed = 3.0f;
  bsplineGenerator.init(footFrontContour, static_cast<float>(kickDuration), kickSpeed);

  // Initialize motions
  StandSupportLegPhase standSupportLegPhase(1001);
  StandSupportLegPhase finishedPhase(1001);
  finishedPhase.phaseId = BKickPhase::finished;

  KickFootBezierPhase bsStrikeOut(200 /* ignored */, bsplineGenerator, request, BKickPhase::strikeoutKickFoot);
  KickFootBezierPhase bsKick(100 /* ignored */, bsplineGenerator, request, BKickPhase::bezierKickFoot);
  KickFootBezierPhase bsBallContact(50 /* ignored */, bsplineGenerator, request, BKickPhase::ballContact);
  KickFootBezierPhase bsSwingOut(70 /* ignored */, bsplineGenerator, request, BKickPhase::swingOut);
  KickFootBezierPhase bsLowerFoot(70 /* ignored */, bsplineGenerator, request, BKickPhase::lowerKickFoot);
  const BKickPhase *bsplineKickPhases[7] = {&standSupportLegPhase, &bsStrikeOut, &bsKick, &bsBallContact, &bsSwingOut, &bsLowerFoot, &finishedPhase};
  kicks[IndykickRequest::bsplineKick] = BKick(7, bsplineKickPhases);
}

void IndykickEngine::update(IndykickEngineOutput& indykickEngineOutput)
{
  /* Arm motion calculation plots. */
  DECLARE_PLOT("module:IndykickEngine:zTorque");
  DECLARE_PLOT("module:IndykickEngine:deltaTau2");
  DECLARE_PLOT("module:IndykickEngine:deltaTau1");
  DECLARE_PLOT("module:IndykickEngine:deltaTau");
  DECLARE_PLOT("module:IndykickEngine:deltaTauAverage");
  DECLARE_PLOT("module:IndykickEngine:leftArmPosition");
  /* CoM motion plots relevant to cart table model. */
  DECLARE_PLOT("module:IndykickEngine:comPositionX");
  DECLARE_PLOT("module:IndykickEngine:comPositionY");
  DECLARE_PLOT("module:IndykickEngine:comAccelerationX");
  DECLARE_PLOT("module:IndykickEngine:comAccelerationY");
  /* Cart table model plots. */
  DECLARE_PLOT("module:IndykickEngine:carttable:zmpError");
  DECLARE_PLOT("module:IndykickEngine:carttable:comPositionError");
  DECLARE_PLOT("module:IndykickEngine:carttable:comVelocityError");
  DECLARE_PLOT("module:IndykickEngine:carttable:comAccelerationError");
  /* Plots for kick foot orientation correction. */
  DECLARE_PLOT("module:IndykickEngine:carttable:kickfoot:rotation:x");
  DECLARE_PLOT("module:IndykickEngine:carttable:kickfoot:rotation:y");
  DECLARE_PLOT("module:IndykickEngine:carttable:kickfoot:rotation:z");
  DECLARE_PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:x");
  DECLARE_PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:y");
  DECLARE_PLOT("module:IndykickEngine:carttable:correctedkickfoot:rotation:z");
  /* Plots to show torso bearing error. */
  DECLARE_PLOT("module:IndykickEngine:carttable:orientationerror:y");
  DECLARE_PLOT("module:IndykickEngine:carttable:demandedxzrotation");
  DECLARE_PLOT("module:IndykickEngine:carttable:measuredxzrotation");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:kickFootPose", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:kickFootTargetPose", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:kickFootTargetPose2", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:contactPoint", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:targetContactPoint", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:ballContactPoint", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:kickFootTrajectory", "robot");
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:rightfootcontour", "RFoot", drawRightFootContour(););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:deboorRight", "RHip1",
                          bsplineGenerator.drawDeBoorPoints(););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:deboorLeft", "LHip1",
                          bsplineGenerator.drawDeBoorPoints(););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:secondsegment", "RHip1",
                          bsplineGenerator.drawPointOnSplineWithTangent(1, 0.75););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:segmentMax", "RHip1", bsplineGenerator.drawMaximumOfSegment(1););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:curveReference", "RHip1", bsplineGenerator.drawReferencePoints(););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:strikeOutBallLeft", "LFoot", bsplineGenerator.drawStrikeOutBall(true););
  DECLARE_DEBUG_DRAWING3D("module:IndykickEngine:strikeOutBallRight", "RFoot", bsplineGenerator.drawStrikeOutBall(false););

  MODIFY("parameters:bsplinegenerator", bsplineGenerator.p);

  output.useGyroCorrection = false;

  if(!kickInProgress)
  {
    if(theMotionRequest.motion != MotionRequest::indykick || !isKickRequestFeasible(theMotionRequest.indykickRequest))
    {
      /* Do not kick if no kick is requested or the request is infeasible. */
      lastMotionStand = theMotionRequest.motion == MotionRequest::stand;
      indykickEngineOutput.isLeavingPossible = true;
      return;
    }

    if(lastMotionStand)
      initializeKickEngineFromStand();
  }

  // TODO: Support other transitions, too?
  ASSERT(lastMotionStand || kickInProgress || theMotionRequest.motion == MotionRequest::indykick);

  if(abs(motionCycleTime / 1000.0f - theFrameInfo.cycleTime) > 0.001f)
    OUTPUT_WARNING("Motion cycle time in the frame info is "
                   << theFrameInfo.cycleTime
                   << "s. Indykick is configured to work with "
                   << (motionCycleTime / 1000.0f) << "s motion cycles.");

  output.isLeavingPossible = false;

  futureBodyDynamicsProviderDelegate.update(futureBodyDynamics, theFutureJointDynamics);
  futureBodyDynamicsProviderDelegate.update(futureBodyDynamicsDerivatives, futureBodyDynamics, theFutureJointDynamics);

  // Make sure the body dynamics have been calculated for the support leg we are going to stand on.
  ASSERT(request.supportLeg == futureBodyDynamics.supportLeg);

  if(comHeightReached)
  {
    // Execute the current request.
    if(request.motion < IndykickRequest::none)
    {
      BKick& kick = kicks[request.motion];
      updateZmpPreview(kick, Vector2f(wantedZmp.x, wantedZmp.y));

      BKickPhase& phase = kick.currentPhase();
      bool next = false;
      switch(phase.phaseId)
      {
        case BKickPhase::finished:
          next = phaseFinished(phase);
          break;
        case BKickPhase::uninitialized:
          ASSERT(false);
          break;
        default:
          next = playPhase(phase);
      }
      if(next)
      {
        if(kick.currentPhase().phaseId == BKickPhase::lowerKickFoot && kickFootPose.reference == KickFootPose::origin)
        {
          // (TODO remove) Hack: The next phase wants the kick foot pose in support foot coordinates.
          const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left ? MassCalibration::footLeft : MassCalibration::footRight;
          Pose3D kickFootInOrigin = kickFootPose.pose;
          kickFootPose.pose = theRobotModel.limbs[supportFoot].invert();
          kickFootPose.pose.conc(kickFootInOrigin);
          kickFootPose.reference = KickFootPose::supportFoot;
        }
        kick.nextPhase(kickFootPose);
        lastPhaseChange = theFrameInfo.time;
        if(kick.currentPhase().phaseId == BKickPhase::finished)
          generateRetractArmCurve();
      }
    }
  }
  else
  {
    comHeightReached = interpolateComHeight();
    lastPhaseChange = theFrameInfo.time;
  }

  indykickEngineOutput = output;
  indykickEngineOutput.executedIndykickRequest = request;
  lastMotionStand = false;

  Pose3D kickFootPoseInSupportFoot;
  if(kickFootPose.reference == KickFootPose::supportFoot)
    kickFootPoseInSupportFoot = kickFootPose.pose;
  else
  {
    ASSERT(kickFootPose.reference == KickFootPose::origin);
    const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left ? MassCalibration::footLeft : MassCalibration::footRight;
    kickFootPoseInSupportFoot = theRobotModel.limbs[supportFoot].invert();
    kickFootPoseInSupportFoot.conc(kickFootPose.pose);
  }

  PLOT("module:IndykickEngine:leftArmPosition", leftArmPosition);

  PLOT("module:IndykickEngine:kickHipPitchAngleCommand", toDegrees(indykickEngineOutput.angles[kickLegJoint0 + 2]));

  // Disable all joints on emergency off
  if(theKeyStates.pressed[KeyStates::chest])
    emergency = true;
  if(emergency)
    for(int i = 0; i < JointData::numOfJoints; ++i)
      indykickEngineOutput.angles[i] = JointData::off;
}

bool IndykickEngine::isKickRequestFeasible(const IndykickRequest& kickRequest) const
{
  bool ballPositionFeasible = kickRequest.ballPosition.x < 200.0f;
  ballPositionFeasible &= kickRequest.ballPosition.x > 150.0f;
  ballPositionFeasible &= abs(kickRequest.ballPosition.y) < 75.0f;
  ballPositionFeasible &= abs(kickRequest.ballPosition.y) > 25.0f;

  bool supportLegCompatible = kickRequest.supportLeg == IndykickRequest::left && kickRequest.ballPosition.y < 0.0f;
  supportLegCompatible |= kickRequest.supportLeg == IndykickRequest::right && kickRequest.ballPosition.y > 0.0f;

  ballPositionFeasible &= supportLegCompatible;

  return ballPositionFeasible;
}

void IndykickEngine::initializeKickEngineFromStand()
{
  kickInProgress = true;
  request = theMotionRequest.indykickRequest;

  /* Copy estimated arm angles from the last motion as the 'starting point'. */
  output.angles[JointData::LShoulderPitch] = leftArm[0]  = theJointDynamics.joint[JointData::LShoulderPitch][0];
  output.angles[JointData::LElbowYaw]      = leftArm[1]  = theJointDynamics.joint[JointData::LElbowYaw][0];
  output.angles[JointData::LElbowRoll]     = leftArm[2]  = theJointDynamics.joint[JointData::LElbowRoll][0];
  output.angles[JointData::LShoulderRoll]  = leftArm[3]  = theJointDynamics.joint[JointData::LShoulderRoll][0];
  output.angles[JointData::RShoulderPitch] = rightArm[0] = theJointDynamics.joint[JointData::RShoulderPitch][0];
  output.angles[JointData::RElbowYaw]      = rightArm[1] = theJointDynamics.joint[JointData::RElbowYaw][0];
  output.angles[JointData::RElbowRoll]     = rightArm[2] = theJointDynamics.joint[JointData::RElbowRoll][0];
  output.angles[JointData::RShoulderRoll]  = rightArm[3] = theJointDynamics.joint[JointData::RShoulderRoll][0];

  /* Calculate initial leg angles for current robot model. */
  InverseKinematic::calcLegJoints(theRobotModel.limbs[MassCalibration::footLeft],
                                  theRobotModel.limbs[MassCalibration::footRight],
                                  output, theRobotDimensions);

  nonGyroCorrectedRobotModel.setJointData(output, theRobotDimensions, theMassCalibration);
  bsplineGenerator.setDuration(request.kickDuration);
  bsplineGenerator.referenceCoordinateSystem = useSupportFootReference ? KickFootPose::supportFoot : KickFootPose::origin;
  wantedZmp.x = zmp.x;
  wantedZmp.y = request.supportLeg == IndykickRequest::left ? -zmp.y : zmp.y;
  zmpReference.init();
  const bool leftSupportLeg = request.supportLeg == IndykickRequest::left;
  const MassCalibration::Limb kickFoot    = leftSupportLeg ? MassCalibration::footRight
                                                           : MassCalibration::footLeft;
  const MassCalibration::Limb supportFoot = leftSupportLeg ? MassCalibration::footLeft
                                                           : MassCalibration::footRight;
  // Stand position of the kick (non-support) foot relative to the support foot.
  const Pose3D originInSupportFoot = theRobotModel.limbs[supportFoot].invert();
  Pose3D kickFootInSupportFoot(originInSupportFoot);
  kickFootInSupportFoot.conc(theRobotModel.limbs[kickFoot]);
  kickFootPose = KickFootPose(kickFootInSupportFoot, KickFootPose::supportFoot);
  initialKickPositionReached = false;
  const Vector3<> comInSupportFoot = originInSupportFoot * theRobotModel.centerOfMass;
  standZmp = Vector3<>(comInSupportFoot.x, comInSupportFoot.y, 0.0f);
  if(request.motion < IndykickRequest::none)
  {
    kicks[request.motion].reset(kickFootPose, Vector2f(standZmp.x, standZmp.y), Vector2f(wantedZmp.x, wantedZmp.y));
  }
  lastPhaseChange = theFrameInfo.time;

  supportLegJoint0 = leftSupportLeg ? JointData::LHipYawPitch
                                    : JointData::RHipYawPitch;
  kickLegJoint0 = leftSupportLeg ? JointData::RHipYawPitch
                                 : JointData::LHipYawPitch;

  leftArm[0] = output.angles[JointData::LShoulderPitch];
  leftArm[1] = output.angles[JointData::LElbowYaw];
  leftArm[2] = output.angles[JointData::LElbowRoll];
  leftArm[3] = output.angles[JointData::LShoulderRoll];
  rightArm[0] = output.angles[JointData::RShoulderPitch];
  rightArm[1] = output.angles[JointData::RElbowYaw];
  rightArm[2] = output.angles[JointData::RElbowRoll];
  rightArm[3] = output.angles[JointData::RShoulderRoll];
  armVelocitiesDifferentiator[0].prediction = leftArm[0];
  armVelocitiesDifferentiator[1].prediction = leftArm[2];
  armVelocitiesDifferentiator[2].prediction = rightArm[0];
  armVelocitiesDifferentiator[3].prediction = rightArm[2];
  for(int i = 0; i < 4; ++i)
  {
    armVelocitiesDifferentiator[i].derivative = 0.0f;
    armVelocitiesDifferentiator[i].derivative2 = 0.0f;
  }

  leftArmPosition = 0.5f;
  initialLeftArmCurve.p0[0] = leftArm[0];
  initialLeftArmCurve.p0[1] = leftArm[1];
  initialLeftArmCurve.p0[2] = leftArm[2];
  initialLeftArmCurve.p0[3] = leftArm[3];
  initialLeftArmCurve.p1 = initialLeftArmCurve.p0;
  initialLeftArmCurve.p3[0] = (armBack.x + armFront.x) * leftArmPosition;
  initialLeftArmCurve.p3[1] = elbowYaw;
  initialLeftArmCurve.p3[2] = (armBack.y + armFront.y) * leftArmPosition;
  initialLeftArmCurve.p3[3] = shoulderRoll;
  initialLeftArmCurve.p2 = initialLeftArmCurve.p3;
  initialRightArmCurve = initialLeftArmCurve;
  /* Add some extra angle to the shoulder roll on the support leg. */
  const float supportLegExtraAngle = fromDegrees(30.0f);
  if(leftSupportLeg)
  {
    initialLeftArmCurve.p3[3] += supportLegExtraAngle;
    initialLeftArmCurve.p2[3] += supportLegExtraAngle;
  }
  else
  {
    initialRightArmCurve.p3[3] += supportLegExtraAngle;
    initialRightArmCurve.p2[3] += supportLegExtraAngle;
  }

  /* Initially there's no arm motion due to balancing. */
  deltaTau = deltaTau1 = 0.0f;

  /* Set up center of mass height interpolation. */
  comHeightReached = false;
  comHeightInterpolationTime = 0.0f;
  comHeightCurve.p0 = comInSupportFoot.z;
  comHeightCurve.p3 = comHeight;
  comHeightCurve.p1 = comHeightCurve.p0;
  comHeightCurve.p2 = comHeightCurve.p3;

  const Pose3D originInKickFoot = theRobotModel.limbs[kickFoot].invert();
  originXCurve.p0 = (originInKickFoot.translation.x + originInSupportFoot.translation.x) / 2.0f;
  originXCurve.p1 = originXCurve.p0;
  originXCurve.p2 = originXCurve.p0 - 20.0f;
  originXCurve.p3 = originXCurve.p2;
  comX = comInSupportFoot.x;

  /* Reset gyro joint velocities corrector. */
  demandedAngularVelocityBuffer.init();
  rotationErrorSum = Vector2<>();
  lastRotationError = Vector2<>();

  cartTableControllerNeedsReset = true;
}

void IndykickEngine::generateRetractArmCurve()
{
  /* Beginning of the initial arm curve is the end of the retraction curve. */
  retractLeftArmCurve.p3 = initialLeftArmCurve.p0;
  retractLeftArmCurve.p2 = initialLeftArmCurve.p1;
  retractRightArmCurve.p3 = initialRightArmCurve.p0;
  retractRightArmCurve.p2 = initialRightArmCurve.p1;

  /* The starting point of a retraction curve is the current arm position. */
  retractLeftArmCurve.p0[0] = output.angles[JointData::LShoulderPitch];
  retractLeftArmCurve.p0[1] = output.angles[JointData::LElbowYaw];
  retractLeftArmCurve.p0[2] = output.angles[JointData::LElbowRoll];
  retractLeftArmCurve.p0[3] = output.angles[JointData::LShoulderRoll];
  retractLeftArmCurve.p1 = retractLeftArmCurve.p0;
  retractRightArmCurve.p0[0] = output.angles[JointData::RShoulderPitch];
  retractRightArmCurve.p0[1] = output.angles[JointData::RElbowYaw];
  retractRightArmCurve.p0[2] = output.angles[JointData::RElbowRoll];
  retractRightArmCurve.p0[3] = output.angles[JointData::RShoulderRoll];
  retractRightArmCurve.p1 = retractRightArmCurve.p0;
}

bool IndykickEngine::interpolateComHeight()
{
  bool reachable;
  comHeightInterpolationTime += motionCycleTime;
  const float progress = std::min(comHeightInterpolationTime / comHeightInterpolationDuration, 1.0f);
  float demandedComHeight = 0.0;
  comHeightCurve.evaluate(progress, demandedComHeight);
  float demandedOriginX = 0.0f;
  originXCurve.evaluate(progress, demandedOriginX);
  RobotModel myRobotModel(output, theRobotDimensions, theMassCalibration);

  Pose3D& leftFootInOrigin = myRobotModel.limbs[MassCalibration::footLeft];
  Pose3D& rightFootInOrigin = myRobotModel.limbs[MassCalibration::footRight];
  leftFootInOrigin.rotation = RotationMatrix(leftFootInOrigin.rotation.getAngleAxis());
  rightFootInOrigin.rotation = RotationMatrix(rightFootInOrigin.rotation.getAngleAxis());
  Pose3D originInLeftFoot = myRobotModel.limbs[MassCalibration::footLeft].invert();
  Pose3D originInRightFoot = myRobotModel.limbs[MassCalibration::footRight].invert();
  Vector3<> demandedComInLeftFoot = originInLeftFoot * myRobotModel.centerOfMass;
  demandedComInLeftFoot.z = demandedComHeight;
  demandedComInLeftFoot.x = comX;

  originInLeftFoot.translation.x = demandedOriginX;
  originInRightFoot.translation.x = demandedOriginX;
  leftFootInOrigin = originInLeftFoot.invert();
  rightFootInOrigin = originInRightFoot.invert();
  reachable = InverseKinematic::calcLegJoints(leftFootInOrigin, rightFootInOrigin, output, theRobotDimensions);
  ASSERT(reachable);
  myRobotModel.setJointData(output, theRobotDimensions, theMassCalibration);
  leftFootInOrigin.rotation = RotationMatrix(leftFootInOrigin.rotation.getAngleAxis());
  rightFootInOrigin.rotation = RotationMatrix(rightFootInOrigin.rotation.getAngleAxis());
  originInLeftFoot = leftFootInOrigin.invert();
  originInRightFoot = rightFootInOrigin.invert();

  Vector3<> comInLeftFoot = originInLeftFoot * myRobotModel.centerOfMass;
  float z2 = sqrt(sqr(comInLeftFoot.x) - sqr(demandedComInLeftFoot.x) + sqr(comInLeftFoot.z));
  float comAngle = atan2(comInLeftFoot.x, comInLeftFoot.z);
  float demandedComAngle = atan2(demandedComInLeftFoot.x, z2);
  float angle = demandedComAngle - comAngle;
  while(abs(angle) > 0.01f)
  {
    originInLeftFoot.rotateY(angle);
    originInRightFoot.rotateY(angle);
    originInLeftFoot.rotation = RotationMatrix(originInLeftFoot.rotation.getAngleAxis());
    originInRightFoot.rotation = RotationMatrix(originInRightFoot.rotation.getAngleAxis());
    leftFootInOrigin = originInLeftFoot.invert();
    rightFootInOrigin = originInRightFoot.invert();
    reachable = InverseKinematic::calcLegJoints(leftFootInOrigin, rightFootInOrigin, output, theRobotDimensions);
    ASSERT(reachable);
    myRobotModel.setJointData(output, theRobotDimensions, theMassCalibration);
    leftFootInOrigin.rotation = RotationMatrix(leftFootInOrigin.rotation.getAngleAxis());
    rightFootInOrigin.rotation = RotationMatrix(rightFootInOrigin.rotation.getAngleAxis());
    originInLeftFoot = leftFootInOrigin.invert();
    originInRightFoot = rightFootInOrigin.invert();
    comInLeftFoot = originInLeftFoot * myRobotModel.centerOfMass;
    z2 = sqrt(sqr(comInLeftFoot.x) - sqr(demandedComInLeftFoot.x) + sqr(comInLeftFoot.z));
    comAngle = atan2(comInLeftFoot.x, comInLeftFoot.z);
    demandedComAngle = atan2(demandedComInLeftFoot.x, z2);
    angle = demandedComAngle - comAngle;
  }

  /* Correct the z coordinate. */
  for(int i = 0; i < 7; ++i)
  {
    const float deltaComHeight = demandedComHeight - comInLeftFoot.z;
    if(abs(deltaComHeight) < 0.00005f)
      break;
    originInLeftFoot.translation.z += deltaComHeight;
    originInRightFoot.translation.z += deltaComHeight;
    leftFootInOrigin = originInLeftFoot.invert();
    rightFootInOrigin = originInRightFoot.invert();
    InverseKinematic::calcLegJoints(leftFootInOrigin, rightFootInOrigin, output, theRobotDimensions);
    myRobotModel.setJointData(output, theRobotDimensions, theMassCalibration);
    comInLeftFoot = originInLeftFoot * myRobotModel.centerOfMass;
  }

  if (progress >= 1.0f)
  {
    /* Update leg joint dynamics for the controllers. */
    for(int i = 0; i < numOfLegJoints; ++i)
    {
      kickLegVelocitiesDifferentiator[i].prediction = output.angles[kickLegJoint0 + i];; // First prediction of the kick leg angles.
      kickLegVelocitiesDifferentiator[i].derivative = 0.0f;
      kickLegVelocitiesDifferentiator[i].derivative2 = 0.0f;
    }
    return true;
  }
  return false;
}

bool IndykickEngine::phaseFinished(const BKickPhase& phase)
{
  lastEstimatedZmp = estimatedZmp;
  calculateZmp(estimatedZmp, futureBodyDynamics);
  const unsigned phaseRuntime = theFrameInfo.getTimeSince(lastPhaseChange);
  phase.seek(phaseRuntime, kickFootPose);
  const int duration = phase.getDuration();
  ASSERT(duration > 0);
  const float armProgress = max(0.0f, min(static_cast<float>(phaseRuntime) / static_cast<float>(duration), 1.0f));
  retractLeftArmCurve.evaluate(armProgress, leftArm);
  retractRightArmCurve.evaluate(armProgress, rightArm);
  balance(false, true);
  kickInProgress = !isBalancedAndStable(radiusZmp, true);
  return false;
}

bool IndykickEngine::isBalancedAndStable(const float radius, const bool stand) const
{
  const Vector3<>& zmp = stand ? standZmp : wantedZmp;
  const float maxRadiusSqr = sqr(radius);
  const float radiusSqr = (estimatedZmp - zmp).squareAbs();
  const float speed = (estimatedZmp - lastEstimatedZmp).abs();
  return (radiusSqr < maxRadiusSqr) && (speed < maxStableZmpSpeed);
}

bool IndykickEngine::playPhase(const BKickPhase& phase)
{
  const unsigned phaseRuntime = theFrameInfo.getTimeSince(lastPhaseChange);
  bool completed = phase.seek(phaseRuntime, kickFootPose);
  drawKickFootDrawings(phase);

  lastEstimatedZmp = estimatedZmp;
  calculateZmp(estimatedZmp, futureBodyDynamics);
  calculatePartialZmpDerivativesByJointMotion();
//  calculateCartTableZMP();

  if(phase.phaseId == BKickPhase::supportLegStand)
  {
    const float progress = min(static_cast<float>(phaseRuntime) / 1001.0f, 1.0f);
    initialLeftArmCurve.evaluate(progress, leftArm);
    initialRightArmCurve.evaluate(progress, rightArm);
  }

  const bool balanceWithArms = phase.phaseId > BKickPhase::liftKickFoot;
  const bool balanceWithTorsoRotation = phase.phaseId == BKickPhase::supportLegStand
    && phase.phaseId < BKickPhase::finished;
  balance(balanceWithArms, balanceWithTorsoRotation);

  if(phase.phaseId == BKickPhase::supportLegStand)
    completed = isBalancedAndStable(radiusZmp, false);
  return completed;
}


void IndykickEngine::calculatePartialZmpDerivativesByJointMotion()
{
  const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left
                                          ? MassCalibration::footLeft
                                          : MassCalibration::footRight;
  const SpatialVector<>& footForce = futureBodyDynamics.limbs[supportFoot].fc;
  const SpatialVector<> groundForce = futureBodyDynamicsProviderDelegate.supportFootInGround * footForce;
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    zmpByq[i] = zmpDerivativeFromFootForceDerivative(groundForce, futureBodyDynamicsDerivatives.partialDerivatives[i].fcByqGround);
    zmpByqDot[i] = zmpDerivativeFromFootForceDerivative(groundForce, futureBodyDynamicsDerivatives.partialDerivatives[i].fcByqDotGround);
    zmpByqDoubleDot[i] = zmpDerivativeFromFootForceDerivative(groundForce, futureBodyDynamicsDerivatives.partialDerivatives[i].fcByqDoubleDotGround);
  }
}

void IndykickEngine::balance(const bool withArms, const bool balanceWithTorsoRotation)
{
  const bool leftSupport = request.supportLeg == IndykickRequest::left;
  const MassCalibration::Limb supportFoot = leftSupport ? MassCalibration::footLeft  : MassCalibration::footRight;

  const Vector3<> zmpWithoutKickMotion = estimatedZmp;

  /*
   * Use the current joint dynamics here to get estimated joint data and not the future joint dynamics.
   * The prediction jumps too much, so I rather have an a little inaccurate
   * robot model (its old!) which leads to a less shaky kick leg motion.
   */
  JointDynamics myFutureJointDynamics = theFutureJointDynamics;
  if(theHeadJointRequest.pan != JointData::off && theHeadJointRequest.pan != JointData::ignore)
    myFutureJointDynamics.joint[JointData::HeadYaw][0] = theHeadJointRequest.pan;
  if(theHeadJointRequest.tilt != JointData::off && theHeadJointRequest.tilt != JointData::ignore)
    myFutureJointDynamics.joint[JointData::HeadPitch][0] = theHeadJointRequest.tilt;
  JointData& outputJointData = output;
  outputJointData = theJointDynamics.asJointData();
  const RobotModel myRobotModel(outputJointData, theRobotDimensions, theMassCalibration);

  if(withArms)
  {
    /* Calculate the expected leg angles. */
    for(int j = 1; j < numOfLegJoints; ++j)
    {
      Vector3f& futureJoint = myFutureJointDynamics.joint[kickLegJoint0 + j];
      Differentiator kvd = kickLegVelocitiesDifferentiator[j];
      futureJoint[0] += motionCycleTime * kvd.derivative + sqr(motionCycleTime) / 2.0f * kvd.derivative2;
      futureJoint[0] = normalize(futureJoint[0]);
      kvd.update(futureJoint[0]);
      futureJoint[1] = kvd.derivative;
      futureJoint[2] = kvd.derivative2;
    }

    /* Calculate expected arm angles. */
    const JointData::Joint armBalanceJoints[4] = {JointData::LShoulderPitch, JointData::LElbowRoll,
      JointData::RShoulderPitch, JointData::RElbowRoll};
    const float slopes[4] = {armFront.x - armBack.x, // left shoulder slope
      armFront.y - armBack.y,  // left elbow slope
      armBack.x - armFront.x,  // - (left shoulder slope)
      armBack.y - armFront.y}; // - (left elbow slope)

    for(int j = 0; j < 4; ++j)
    {
      Vector3f& futureJoint = myFutureJointDynamics.joint[armBalanceJoints[j]];
      Differentiator avd = armVelocitiesDifferentiator[j];

      float newAngle = futureJoint[0];
      newAngle += avd.derivative * motionCycleTime;
      newAngle += avd.derivative2 / 2.0f * sqr(motionCycleTime);
      newAngle = normalize(newAngle);
      avd.update(newAngle);
      futureJoint[0] = newAngle;
      futureJoint[1] = avd.derivative;
      futureJoint[2] = avd.derivative2;
    }

    /* Calculate the delta of the vertical torque given uniformly accelerated motion of arms and legs. */
    futureBodyDynamicsProviderDelegate.invalidateBodyInSupportFootTransformations();
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamics, myFutureJointDynamics);
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamicsDerivatives, futureBodyDynamics, myFutureJointDynamics);
    calculateZmp(estimatedZmp, futureBodyDynamics);
    calculatePartialZmpDerivativesByJointMotion();
    float zTorqueWithLegArmMotion = futureBodyDynamics.limbs[supportFoot].fc.top.z;
    PLOT("module:IndykickEngine:zTorque", zTorqueWithLegArmMotion);

    /* Convert the kick foot pose from support foot coordinates into origin coordinates and update kick leg angles. */
    if(kickFootPose.reference == KickFootPose::origin)
      InverseKinematic::calcLegJoints(kickFootPose.pose, outputJointData, !leftSupport, theRobotDimensions);
    else
    {
      Pose3D kickFootInOrigin(myRobotModel.limbs[supportFoot]);
      kickFootInOrigin.conc(kickFootPose.pose);
      InverseKinematic::calcLegJoints(kickFootInOrigin, outputJointData, !leftSupport, theRobotDimensions);
    }

    /* Add the delta of the vertical torque caused by the deviation from the uniformly accelerated motion. */
    for(int j = 1; j < numOfLegJoints; ++j)
    {
      const float deltaq = normalize(outputJointData.angles[kickLegJoint0 + j] - myFutureJointDynamics.joint[kickLegJoint0 + j][0]);
      zTorqueWithLegArmMotion += zTorqueGradient(static_cast<JointData::Joint>(kickLegJoint0 + j)) * deltaq;
    }

    /* Calculate the vertical torque derivative by tau. */
    float zTorqueByTau = 0.0f;
    for(int j = 0; j < 4; ++j)
      zTorqueByTau += zTorqueGradient(armBalanceJoints[j]) * slopes[j];
    const float deltaTau2 = -zTorqueWithLegArmMotion * armBalanceGain / zTorqueByTau;

    const float smoothing = 0.1f;
    deltaTau1 = smoothing * deltaTau2 + (1.0f - smoothing) * deltaTau1;
    deltaTau = smoothing * deltaTau1 + (1.0f - smoothing) * deltaTau;

    PLOT("module:IndykickEngine:deltaTau2", deltaTau2);
    PLOT("module:IndykickEngine:deltaTau1", deltaTau1);
    deltaTau = smoothing * deltaTau2 + (1.0f - smoothing) * deltaTau;
    PLOT("module:IndykickEngine:deltaTau", deltaTau);
    const float deltaTauAverage = 2.0f * deltaTau - deltaTau1;
    PLOT("module:IndykickEngine:deltaTauAverage", deltaTauAverage);

    /* Generate the new arm angles and update the differentiators and the future joint dynamics. */
    for(int j = 0; j < 4; ++j)
    {
      Vector3f& futureJoint = myFutureJointDynamics.joint[armBalanceJoints[j]];
      Differentiator& avd = armVelocitiesDifferentiator[j];

      futureJoint[0] += slopes[j] * deltaTauAverage;
      futureJoint[0] = normalize(futureJoint[0]);
      avd.update(futureJoint[0]);
      futureJoint[1] = avd.derivative;
      futureJoint[2] = avd.derivative2;
    }

    /* Update the future joint dynamics of the kick leg. */
    for(int j = 1; j < numOfLegJoints; ++j)
    {
      Differentiator kvd = kickLegVelocitiesDifferentiator[j];
      kvd.update(outputJointData.angles[kickLegJoint0 + j]);
      Vector3f& futureJoint = myFutureJointDynamics.joint[kickLegJoint0 + j];
      futureJoint[0] = outputJointData.angles[kickLegJoint0 + j];
      futureJoint[1] = kvd.derivative;
      futureJoint[2] = kvd.derivative2;
    }

    leftArm[0] = myFutureJointDynamics.joint[JointData::LShoulderPitch][0];
    leftArm[2] = myFutureJointDynamics.joint[JointData::LElbowRoll][0];
    rightArm[0] = myFutureJointDynamics.joint[JointData::RShoulderPitch][0];
    rightArm[2] = myFutureJointDynamics.joint[JointData::RElbowRoll][0];

    /* Update the body dynamics and the zmp and zmp derivative
     * with the new arm angles and perform the balancing on these dynamics */
    futureBodyDynamicsProviderDelegate.invalidateBodyInSupportFootTransformations();
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamics, myFutureJointDynamics);
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamicsDerivatives, futureBodyDynamics, myFutureJointDynamics);
    calculateZmp(estimatedZmp, futureBodyDynamics);
    calculatePartialZmpDerivativesByJointMotion();
  }
  else // If we're not balancing with the arms here, we have to use the arm angles set by the calling function, since they have to be included in the zmp/com calculations.
  {
    const JointData::Joint leftArmJoints[4] = {JointData::LShoulderPitch, JointData::LElbowYaw, JointData::LElbowRoll, JointData::LShoulderRoll};
    const JointData::Joint rightArmJoints[4] = {JointData::RShoulderPitch, JointData::RElbowYaw, JointData::RElbowRoll, JointData::RShoulderRoll};
    for(int j = 0; j < 4; ++j)
    {
      Vector3f& futureLeftArmJoint = myFutureJointDynamics.joint[leftArmJoints[j]];
      Vector3f& futureRightArmJoint = myFutureJointDynamics.joint[rightArmJoints[j]];

      futureLeftArmJoint[0] = normalize(leftArm[j]);
      futureRightArmJoint[0] = normalize(rightArm[j]);
      // TODO: Also update arm velocities and accelerations. Not done yet, because they are very small if we're not balancing with the arms.
    }

    futureBodyDynamicsProviderDelegate.invalidateBodyInSupportFootTransformations();
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamics, myFutureJointDynamics);
    futureBodyDynamicsProviderDelegate.update(futureBodyDynamicsDerivatives, futureBodyDynamics, myFutureJointDynamics);
  }

  /* Generate new leg joint angles. */
  Vector2<> demandedTorsoAngularVelocity;
  balanceZmpObjectiveCartTable(myFutureJointDynamics, balanceWithTorsoRotation, zmpWithoutKickMotion, demandedTorsoAngularVelocity);

  /* Update the kick leg differentiators. */
  for(int j = 0; j < numOfLegJoints; ++j)
    kickLegVelocitiesDifferentiator[j].update(outputJointData.angles[kickLegJoint0 + j]);

  /* Update head angles. */
  outputJointData.angles[JointData::HeadYaw] = myFutureJointDynamics.joint[JointData::HeadYaw][0];
  outputJointData.angles[JointData::HeadPitch]  = myFutureJointDynamics.joint[JointData::HeadPitch][0];

  /* Apply updated arm angles. */
  outputJointData.angles[JointData::LShoulderPitch] = leftArm[0];
  outputJointData.angles[JointData::LElbowYaw] = leftArm[1];
  outputJointData.angles[JointData::LElbowRoll] = leftArm[2];
  outputJointData.angles[JointData::LShoulderRoll] = leftArm[3];
  outputJointData.angles[JointData::RShoulderPitch] = rightArm[0];
  outputJointData.angles[JointData::RElbowYaw] = rightArm[1];
  outputJointData.angles[JointData::RElbowRoll]= rightArm[2];
  outputJointData.angles[JointData::RShoulderRoll] = rightArm[3];

  nonGyroCorrectedRobotModel.setJointData(outputJointData, theRobotDimensions, theMassCalibration);

  output.useGyroCorrection = true;
  for(int i = 0; i < numOfLegJoints; ++i)
  {
    output.notGyroCorrectedLeftLegAngles[i] = output.angles[JointData::LHipYawPitch + i];
    output.notGyroCorrectedRightLegAngles[i] = output.angles[JointData::RHipYawPitch + i];
  }

  const float kickLegCorrectionOn = 4.0f;
  const float kickLegCorrectionOff = 7.5;

  Pose3D kickFootPoseInSupportFoot;
  if(kickFootPose.reference == KickFootPose::origin)
  {
    kickFootPoseInSupportFoot = theRobotModel.limbs[supportFoot].invert();
    kickFootPoseInSupportFoot.conc(kickFootPose.pose);
  }
  else
  {
    ASSERT(kickFootPose.reference == KickFootPose::supportFoot);
    kickFootPoseInSupportFoot = kickFootPose.pose;
  }

  if(kickFootPoseInSupportFoot.translation.z < kickLegCorrectionOn)
    correctKickLegWithGyro = true;
  else if(kickFootPoseInSupportFoot.translation.z > kickLegCorrectionOff)
    correctKickLegWithGyro = false;
  correctKickLegJointVelocities(demandedTorsoAngularVelocity, correctKickLegWithGyro, outputJointData);

  /* Make leg joints as stiff as possible. */
  for(int j = 0; j < numOfLegJoints; ++j)
  {
    output.jointHardness.hardness[JointData::RHipYawPitch + j] = 100;
    output.jointHardness.hardness[JointData::LHipYawPitch + j] = 100;
  }
}

void IndykickEngine::calculateZmp(Vector3<>& zmp, const BodyDynamics& myBodyDynamics) const
{
  const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left ? MassCalibration::footLeft : MassCalibration::footRight;
  const SpatialVector<>& footForce = myBodyDynamics.limbs[supportFoot].fc;
  const SpatialVector<> groundForce = futureBodyDynamicsProviderDelegate.supportFootInGround * footForce;
  zmp.x = -groundForce.top.y / groundForce.bottom.z;
  zmp.y = groundForce.top.x / groundForce.bottom.z;
  zmp.z = 0.0f;
}

void IndykickEngine::balanceZmpObjectiveCartTable(const JointDynamics& myFutureJointDynamics,
                                               const bool balanceWithTorsoRotation,
                                               const Vector3<>& zmp,
                                               Vector2<>& demandedTorsoAngularVelocity)
{
  JointData myJointData = myFutureJointDynamics.asJointData(); /* Joints for both legs will be overwritten. */
  /* Calculate com. */
  const bool leftSupport = request.supportLeg == IndykickRequest::left;
  const MassCalibration::Limb supportFoot = leftSupport ? MassCalibration::footLeft : MassCalibration::footRight;
  RobotModel myRobotModel(myJointData, theRobotDimensions, theMassCalibration);
  Vector3<> comPosition = myRobotModel.limbs[supportFoot].invert() * myRobotModel.centerOfMass;
  comPosition.z = comHeight;

  /* To do that, calculate com motion in support foot coordinates. */
  Vector3<> comVelocity;
  Vector3<> comAcceleration;
  float totalMass = 0.0f;
  for(int limbIdx = MassCalibration::neck; limbIdx < MassCalibration::numOfLimbs; ++limbIdx)
  {
    const MassCalibration::Limb l = static_cast<MassCalibration::Limb>(limbIdx);
    const Body& body = futureBodyDynamics.limbs[l];
    const SpatialTransform& bodyInSupportFoot = futureBodyDynamicsProviderDelegate.getBodyInSupportFoot(l, futureBodyDynamics);
    const SpatialVector<> velocityInSupportFoot = bodyInSupportFoot * body.v;
    const SpatialVector<> accelerationInSupportFoot = bodyInSupportFoot * body.a;
    const Vector3<> linearAccelerationOffset = velocityInSupportFoot.top ^ velocityInSupportFoot.bottom;
    const Vector3<> linearAcceleration = accelerationInSupportFoot.bottom + linearAccelerationOffset;

    totalMass += body.I.mass;
    comAcceleration += linearAcceleration * body.I.mass;
    comVelocity += velocityInSupportFoot.bottom * body.I.mass;
  }
  comAcceleration /= totalMass;
  comVelocity /= totalMass;
  if(cartTableControllerNeedsReset)
  {
    cartTableController.reset(leftSupport, comPosition, comVelocity, comAcceleration, myRobotModel);
    cartTableControllerNeedsReset = false;
  }

  PLOT("module:IndykickEngine:comPositionX", comPosition.x);
  PLOT("module:IndykickEngine:comPositionY", comPosition.y);
  PLOT("module:IndykickEngine:comAccelerationX", comAcceleration.x);
  PLOT("module:IndykickEngine:comAccelerationY", comAcceleration.y);

  cartTableController.generateSupportLegAngles(balanceWithTorsoRotation ? torsoRotationYBalance : torsoRotationYNoBalance,
                                               balanceWithTorsoRotation ? torsoRotationXBalance : torsoRotationXNoBalance,
                                               zmpReference, zmp,
                                               kickFootPose.pose, kickFootPose.reference == KickFootPose::supportFoot,
                                               myJointData,
                                               comPosition, comVelocity, comAcceleration,
                                               demandedTorsoAngularVelocity);

  for(int j = 0; j < numOfLegJoints; ++j)
  {
    output.angles[supportLegJoint0 + j] = myJointData.angles[supportLegJoint0 + j];
    output.angles[kickLegJoint0 + j] = myJointData.angles[kickLegJoint0 + j];
  }
}


Vector3<> IndykickEngine::zmpDerivativeFromFootForceDerivative(const SpatialVector<>& footForceInGround, const SpatialVector<>& footForceDerivativeInGround) const
{
  const float reaction = footForceInGround.bottom.z;
  const float reactionSqr = sqr(reaction);
  const float reactionDerivative = footForceDerivativeInGround.bottom.z;
  const Vector3<>& minusTorque = footForceInGround.top;
  const Vector3<>& minusTorqueDerivative = footForceDerivativeInGround.top;
  return Vector3<>((-minusTorqueDerivative.y * reaction + minusTorque.y * reactionDerivative) / reactionSqr,
                   (minusTorqueDerivative.x * reaction - minusTorque.x * reactionDerivative) / reactionSqr,
                   0.0f);
}

float IndykickEngine::zTorqueGradient(const JointData::Joint joint)
{
  const BodyDynamicsDerivatives::Derivative& derivative = futureBodyDynamicsDerivatives.partialDerivatives[joint];
  const SpatialVector<> fcByqTotalGround = derivative.fcByqDotGround
                                        + derivative.fcByqDotGround * 2.0f / motionCycleTime
                                        + derivative.fcByqDoubleDotGround * 2.0f / sqr(motionCycleTime);
  return fcByqTotalGround.top.z;
}

void IndykickEngine::approximateFootContour()
{
  InMapFile stream("footFrontContour.cfg");
  ASSERT(stream.exists());
  stream >> footFrontContour;
  footFrontContour.approximate();
}

void IndykickEngine::drawKickFootDrawings(const BKickPhase& phase)
{
  COMPLEX_DRAWING3D("module:IndykickEngine:kickFootPose",
                    {
                      const int supportFoot = supportLegJoint0 + 5;
                      Pose3D kickFootInOrigin;
                      if(kickFootPose.reference == KickFootPose::origin)
                        kickFootInOrigin = kickFootPose.pose;
                      else
                      {
                        Pose3D kickFootInOrigin = theRobotModel.limbs[supportFoot];
                        kickFootInOrigin.conc(kickFootPose.pose);
                      }
                      const Vector3<> trans = kickFootInOrigin.rotation.invert() * kickFootInOrigin.translation;
                      const Vector3<> rot = kickFootInOrigin.rotation.getAngleAxis();
                      ROTATE3D("module:IndykickEngine:kickFootPose", rot.x, rot.y, rot.z);
                      TRANSLATE3D("module:IndykickEngine:kickFootPose", trans.x, trans.y, trans.z);
                      COORDINATES3D("module:IndykickEngine:kickFootPose", 100, 2);
                    });
  COMPLEX_DRAWING3D("module:IndykickEngine:kickFootTargetPose",
                    {
                      const int supportFoot = supportLegJoint0 + 5;
                      KickFootPose targetKickFoot;
                      phase.seek(1.0f, targetKickFoot);
                      Pose3D targetKickFootInOrigin;
                      if(targetKickFoot.reference == KickFootPose::supportFoot)
                        targetKickFootInOrigin = theRobotModel.limbs[supportFoot];
                      targetKickFootInOrigin.conc(targetKickFoot.pose);

                      const Vector3<> trans = targetKickFootInOrigin.rotation.invert() * targetKickFootInOrigin.translation;
                      const Vector3<> rot = targetKickFootInOrigin.rotation.getAngleAxis();
                      ROTATE3D("module:IndykickEngine:kickFootTargetPose", rot.x, rot.y, rot.z);
                      TRANSLATE3D("module:IndykickEngine:kickFootTargetPose", trans.x, trans.y, trans.z);
                      COORDINATES3D("module:IndykickEngine:kickFootTargetPose", 100, 2);
                    });
  COMPLEX_DRAWING3D("module:IndykickEngine:kickFootTargetPose2",
                    {
                      const int supportFoot = supportLegJoint0 + 5;
                      KickFootPose kickFoot;
                      phase.seek(1.0f, kickFoot);
                      const Vector3<> trans = (kickFoot.reference == KickFootPose::supportFoot ? Pose3D(theRobotModel.limbs[supportFoot]).conc(kickFoot.pose)
                                                                                               : kickFoot.pose).translation;
                      SPHERE3D("module:IndykickEngine:kickFootTargetPose2", trans.x, trans.y, trans.z, 5, ColorClasses::red);
                    });

  COMPLEX_DRAWING3D("module:IndykickEngine:contactPoint",
                    {
                      const int supportFoot = supportLegJoint0 + 5;
                      const int kickFoot = kickLegJoint0 + 5;
                      Pose3D kickFootInOrigin;
                      if(kickFootPose.reference == KickFootPose::origin)
                        kickFootInOrigin = kickFootPose.pose;
                      else
                      {
                        ASSERT(kickFootPose.reference == KickFootPose::supportFoot);
                        kickFootInOrigin = theRobotModel.limbs[supportFoot];
                        kickFootInOrigin.conc(kickFootPose.pose);
                      }
                      const Vector3<> trans = kickFootInOrigin.rotation.invert() * kickFootInOrigin.translation;
                      const Vector3<> rot = kickFootInOrigin.rotation.getAngleAxis();
                      ROTATE3D("module:IndykickEngine:contactPoint", rot.x, rot.y, rot.z);
                      TRANSLATE3D("module:IndykickEngine:contactPoint", trans.x, trans.y, trans.z);
                      const Vector3<>& cp = bsplineGenerator.contactPoint;
                      Vector3<> tangent = bsplineGenerator.contactTangent;
                      const Vector3<> tangentStart = cp - tangent;
                      const Vector3<> tangentEnd = cp + tangent;
                      SPHERE3D("module:IndykickEngine:contactPoint", cp.x, cp.y, cp.z, 5, ColorClasses::blue);
                      LINE3D("module:IndykickEngine:contactPoint", tangentStart.x, tangentStart.y, tangentStart.z,
                             tangentEnd.x, tangentEnd.y, tangentEnd.z, 3, ColorClasses::red);

                      const RotationMatrix rotRobotInKickFoot = (theTorsoMatrix.rotation * theRobotModel.limbs[kickFoot].rotation).invert();
                      Vector3<> direction(request.kickDirection.x, request.kickDirection.y, 0.0f);
                      direction = rotRobotInKickFoot * direction;
                      const Vector3<> to = cp + direction.normalize(100.0f);
                      CYLINDERARROW3D("module:IndykickEngine:contactPoint", cp, to, 1, 10, 3, ColorClasses::blue);
                    });
  COMPLEX_DRAWING3D("module:IndykickEngine:targetContactPoint",
                    {
                      const int supportFoot = supportLegJoint0 + 5;

                      KickFootPose targetKickFoot;
                      phase.seek(1.0f, targetKickFoot);
                      Pose3D targetKickFootInOrigin;
                      if(targetKickFoot.reference == KickFootPose::supportFoot)
                        targetKickFootInOrigin = theRobotModel.limbs[supportFoot];
                      targetKickFootInOrigin.conc(targetKickFoot.pose);
                      const Vector3<> trans = targetKickFootInOrigin.rotation.invert() * targetKickFootInOrigin.translation;
                      const Vector3<> rot = targetKickFootInOrigin.rotation.getAngleAxis();
                      ROTATE3D("module:IndykickEngine:targetContactPoint", rot.x, rot.y, rot.z);
                      TRANSLATE3D("module:IndykickEngine:targetContactPoint", trans.x, trans.y, trans.z);
                      const Vector3<>& cp = bsplineGenerator.contactPoint;
                      SPHERE3D("module:IndykickEngine:targetContactPoint", cp.x, cp.y, cp.z, 5, ColorClasses::blue);
                    });
  COMPLEX_DRAWING3D("module:IndykickEngine:ballContactPoint",
                    {
                      const int supportFoot = supportLegJoint0 + 5;
                      const Vector3<>& ball = bsplineGenerator.ballPositionInSupportFoot;
                      const Vector3<> ballInOrigin = theRobotModel.limbs[supportFoot] * ball;
                      SPHERE3D("module:IndykickEngine:ballContactPoint", ballInOrigin.x, ballInOrigin.y, ballInOrigin.z, 5, ColorClasses::orange);
                    });

  COMPLEX_DRAWING3D("module:IndykickEngine:kickFootTrajectory",
  {
    const int durationInt = phase.getDuration();
    if(durationInt > 0)
    {
      const unsigned duration = static_cast<unsigned>(durationInt);
      const int supportFoot = supportLegJoint0 + 5;
      const Pose3D& supportFootInOrigin = theRobotModel.limbs[supportFoot];
      const unsigned samples = 100;
      const unsigned delta = duration / samples;
      Vector3<> from; // Beginning of a line segment.

      KickFootPose kickFoot;
      phase.seek(0u, kickFoot);
      from = (kickFoot.reference == KickFootPose::supportFoot ? Pose3D(supportFootInOrigin).conc(kickFoot.pose)
                                                              : kickFoot.pose).translation;
      for(unsigned i = 1; i < samples; ++i)
      {
        phase.seek(i * delta, kickFoot);
        const Vector3<> to = (kickFoot.reference == KickFootPose::supportFoot ? Pose3D(supportFootInOrigin).conc(kickFoot.pose)
                                                                              : kickFoot.pose).translation;
        LINE3D("module:IndykickEngine:kickFootTrajectory", from.x, from.y, from.z, to.x, to.y, to.z, 5, ColorClasses::blue);
        from = to;
      }
      phase.seek(duration, kickFoot);
      const Vector3<> to = (kickFoot.reference == KickFootPose::supportFoot ? Pose3D(supportFootInOrigin).conc(kickFoot.pose)
                                                                            : kickFoot.pose).translation;
      LINE3D("module:IndykickEngine:kickFootTrajectory", from.x, from.y, from.z, to.x, to.y, to.z, 5, ColorClasses::blue);
    }
  });
}

void IndykickEngine::drawRightFootContour()
{
  footFrontContour.draw3D();
  Vector3<> cp = bsplineGenerator.contactPoint;
  cp.z = -40.0f;
  const Vector3<> tangent = bsplineGenerator.contactTangent;
  const Vector3<> dir(1.0f, -tangent.x / tangent.y, 0.0f);
  const Vector3<> tangentStart = cp - tangent;
  const Vector3<> tangentEnd = cp + tangent;
  const Vector3<> dirEnd = cp + dir * 50.0f;

  SPHERE3D("module:IndykickEngine:rightfootcontour", cp.x, cp.y, cp.z, 5, ColorClasses::blue);
  LINE3D("module:IndykickEngine:rightfootcontour", tangentStart.x, tangentStart.y, tangentStart.z,
         tangentEnd.x, tangentEnd.y, tangentEnd.z, 3, ColorClasses::red);
  CYLINDERARROW3D("module:IndykickEngine:rightfootcontour", cp, dirEnd, 1, 10, 3, ColorClasses::blue);
}

void IndykickEngine::correctKickLegJointVelocities(const Vector2<>& demandedTorsoAngularVelocity,
                                                const bool correctKickLeg,
                                                JointData& outputJointData)
{
  demandedAngularVelocityBuffer.add(demandedTorsoAngularVelocity);
  const bool gyroWorks = theFilteredSensorData.data[SensorData::gyroX] != 0.0f
    && theFilteredSensorData.data[SensorData::gyroY] != 0.0f;
  if(demandedAngularVelocityBuffer.getNumberOfEntries() == frameDelay && gyroWorks)
  {
    const Vector2<>& delayedAngularVelocity = demandedAngularVelocityBuffer[frameDelay - 1];
    gyro.x = theFilteredSensorData.data[SensorData::gyroX]; // * 0.3f + 0.7f * gyro.x;
    gyro.y = theFilteredSensorData.data[SensorData::gyroY]; // * 0.3f + 0.7f * gyro.y;
    const Vector2<> error = gyro - delayedAngularVelocity;
    rotationErrorSum += error;
    const Vector2<> errorDerivative = (error - lastRotationError) / 0.01f;
    lastRotationError = error;

    const Vector2<> orientationOffset(rotationErrorSum.x * torsoVelocityCorrection.integralGain.x
                                        + error.x * torsoVelocityCorrection.proportionalGain.x
                                        + errorDerivative.x * torsoVelocityCorrection.derivativeGain.x,
                                      rotationErrorSum.y * torsoVelocityCorrection.integralGain.y
                                        + error.y * torsoVelocityCorrection.proportionalGain.y
                                        + errorDerivative.y * torsoVelocityCorrection.derivativeGain.y);

    /* Add offsets to joints. */
    const bool left = request.supportLeg == IndykickRequest::left;
    const JointData::Joint sleg0 = left ? JointData::LHipYawPitch : JointData::RHipYawPitch; /* Support leg joint 0. */
    const JointData::Joint kleg0 = left ? JointData::RHipYawPitch : JointData::LHipYawPitch; /* Kick leg joint 0. */
    const float sign = left ? -1.0f : 1.0f;
    outputJointData.angles[sleg0 + 2] += orientationOffset.y; // * 0.01f; /* Hip pitch */
    outputJointData.angles[sleg0 + 4] += orientationOffset.y; // * 0.01f; /* Ankle pitch */
    outputJointData.angles[sleg0 + 1] += sign * orientationOffset.x; // * 0.01f; /* Hip roll */
    outputJointData.angles[sleg0 + 5] += sign * orientationOffset.x; // * 0.01f; /* Ankle roll. */
    if(correctKickLeg)
    {
      outputJointData.angles[kleg0 + 2] += orientationOffset.y; // * 0.01f; /* Hip pitch */
      outputJointData.angles[kleg0 + 4] += orientationOffset.y; // * 0.01f; /* Ankle pitch */
      outputJointData.angles[kleg0 + 1] -= sign * orientationOffset.x; // * 0.01f; /* Hip roll */
      outputJointData.angles[kleg0 + 5] -= sign * orientationOffset.x; // * 0.01f; /* Ankle roll. */
    }
  }
}

void IndykickEngine::updateZmpPreview(const BKick& kick, const Vector2f& defaultZmp)
{
  const unsigned cycleTimeMs = static_cast<unsigned>(motionCycleTime);
  ASSERT(cycleTimeMs > 0);
  const int timeInPhase = theFrameInfo.getTimeSince(lastPhaseChange);
  ASSERT(timeInPhase >= 0);
  if(zmpReference.isEmpty())
    for(int i = 0; i < numPreviewFrames - 1; ++i)
      zmpReference.add(kick.zmpAtFutureFrame(i, timeInPhase, cycleTimeMs, defaultZmp));
  zmpReference.add(kick.zmpAtFutureFrame(numPreviewFrames - 1, timeInPhase, cycleTimeMs, defaultZmp));
  ASSERT(zmpReference.isFilled());
}

void IndykickEngine::update(RobotBalance& robotBalance)
{
  if (!kickInProgress)
    return; // Balance is only estimated if a kick is in progress.

  robotBalance.zmp = Vector2<>(estimatedZmp.x, estimatedZmp.y);
  robotBalance.leftSupport = request.supportLeg == IndykickRequest::left;
  robotBalance.numZmpPreview = min((int) RobotBalance::maxZmpPreview, (int) numPreviewFrames);

  for(int i = 0; i < robotBalance.numZmpPreview; ++i)
  {
    const Vector2f& preview = zmpReference[numPreviewFrames - 1 - i];
    robotBalance.zmpPreview[i] = Vector2<>(preview.x, preview.y);
  }
}

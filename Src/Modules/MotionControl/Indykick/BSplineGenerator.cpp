/**
 * @file BSplineGenerator.h
 * Implementations of methods of the classes to generate a cubic spline in bezier representation.
 * @author Felix Wenk
 */

#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Matrix.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/MotionControl/IndykickRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "BSplineGenerator.h"

BSplineGenerator::BSplineGenerator(const TorsoMatrix& theTorsoMatrix,
                                   const RobotModel& theRobotModel,
                                   const RobotDimensions& theRobotDimensions)
: referenceCoordinateSystem(KickFootPose::supportFoot), ballRadius(32.5f),
  theTorsoMatrix(theTorsoMatrix), theRobotModel(theRobotModel), theRobotDimensions(theRobotDimensions)
{}

void BSplineGenerator::init(const FrontContour& footFrontContour,
                            const float kickDuration,
                            const float defaultKickSpeed)
{
  p.kickHeight = 1.0f; // 7.0f / 5.0f;
  p.footRotationFactor = 0.0f;
  p.firstBSplinePhase = BKickPhase::strikeoutKickFoot;
  p.heightOffset = 5.0f; // 15.0f;
  p.strikeoutHeight = p.kickHeight * ballRadius + p.heightOffset;
  p.circleOffset1 = 1.5f; // 2.8f; // 1.0f;
  p.circleOffset2 = 1.5f; // 1.3f;
  p.strikeoutBallDistanceLimit = 85.0f;
  p.swingoutBallDistanceLimit = 20.0f;
  this->kickDuration = kickDuration;
  this->kickSpeed = defaultKickSpeed;
  this->footFrontContour = footFrontContour;

  // Calculate circle of the sphere (the ball)
  circleOfTheBallRadius = ballRadius * std::sqrt(1.0f - sqr(p.kickHeight - 1.0f));

  for(int i = 0; i < BKickPhase::numOfPhaseIds; ++i)
    durations[i] = -1;
}

void BSplineGenerator::setDuration(const unsigned newDuration)
{
  kickDuration = static_cast<float>(newDuration);
}

void BSplineGenerator::generateEndpoints(const BKickPhase::PhaseId phaseId,
                                         const IndykickRequest& request,
                                         const KickFootPose& standKickFootPose)
{
  ASSERT(phaseId >= BKickPhase::strikeoutKickFoot); /* Strike out obviously must be the first phase of the kick. */
  if(phaseId > BKickPhase::strikeoutKickFoot)
    return; /* Everything has been calculated with the first phase. */

  /*
   * Everything is generated in one go when the first kick phase calls
   * generateStartpoints
   */
  this->request = request;
  /*
   * Since the torso matrix and the robot model are probably pretty inaccurate
   * when the spline is generated, the kick direction and ball coordiates are converted here,
   * when the robot is still in its straight stand position on both feet.
   */
  const bool left = request.supportLeg == IndykickRequest::left;
  const MassCalibration::Limb supportFoot = left ? MassCalibration::footLeft : MassCalibration::footRight;
  Pose3D robotInSupportFoot(theTorsoMatrix);
  robotInSupportFoot.conc(theRobotModel.limbs[supportFoot]);
  robotInSupportFoot = robotInSupportFoot.invert();
  ballPositionInSupportFoot = robotInSupportFoot * Vector3<>(request.ballPosition.x, request.ballPosition.y,
                                                             p.heightOffset + p.kickHeight * ballRadius);
  kickDirectionInSupportFoot = robotInSupportFoot.rotation * Vector3<>(request.kickDirection.x, request.kickDirection.y, 0.0f);
  kickDirectionInSupportFoot.normalize();
  generateBallContactReferencePointsInSupportFoot();
  calculateStrikeoutPointInSupportFoot();
}

void BSplineGenerator::generateStartpoints(const BKickPhase::PhaseId phaseId,
                                           const KickFootPose& initialKickFootPose)
{
  if(phaseId == BKickPhase::strikeoutKickFoot)
    generateSpline(initialKickFootPose);
}

void BSplineGenerator::generateSpline(const KickFootPose& standKickFootPose)
{
  kickSpeed = request.kickDirection.abs();
  generateReferencePoints(standKickFootPose);
  generateDeltasAlt();
  generateDeBoorPointsAlt();
  generateDeltasWithLengths();
  generateDeBoorPointsAlt();

  const float originalKickSpeed = kickSpeed;
  while(refineSpeedDS());
  if(originalKickSpeed != kickSpeed)
    OUTPUT(idText, text, "generateSpline: Changed kick speed from "
           << originalKickSpeed << " to " << kickSpeed << ".");

  for(int i = 0; i < numDeltas; ++i)
    durations[p.firstBSplinePhase + i] = static_cast<int>(delta(i) + 0.5f);
  generateBezierPoints();
}

void BSplineGenerator::generateReferencePoints(const KickFootPose& standKickFootPose)
{
  /*
   * TODO: Precalculate everything but the ball-related reference points for the left and the right leg
   * instead of computing that over and over again.
   */
  const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left
    ? MassCalibration::footLeft : MassCalibration::footRight;
  const MassCalibration::Limb kickPelvis = request.supportLeg == IndykickRequest::left
    ? MassCalibration::pelvisRight : MassCalibration::pelvisLeft;
  Pose3D supportFootInKickPelvis(theRobotModel.limbs[kickPelvis].invert());
  supportFootInKickPelvis.conc(theRobotModel.limbs[supportFoot]);

  /* Print torso pose when generating the spline. */
  const Pose3D originInSupportFoot = theRobotModel.limbs[supportFoot].invert();

  /* Calculate starting pose. */
  generateStartAndEndReferencePoint(standKickFootPose, supportFootInKickPelvis);

  /* Calculate poses when kicking the ball. */
  generateBallContactReferencePoints(supportFootInKickPelvis);

  /* Calculate strike out and swing poses. (Pretty much a clone if the angle phase.) */
  calculateStrikeoutPoint(supportFootInKickPelvis);
}

void BSplineGenerator::generateStartAndEndReferencePoint(const KickFootPose& kickFoot, const Pose3D& supportFootInKickPelvis)
{
  // TODO: Don't convert from origin to pelvis and then later back in the bezier points method. That's not useful work.
  Pose3D kickFootPoseInKickPelvis;
  if(kickFoot.reference == KickFootPose::supportFoot)
  {
    kickFootPoseInKickPelvis = supportFootInKickPelvis;
  }
  else
  {
    ASSERT(kickFoot.reference ==  KickFootPose::origin);
    const MassCalibration::Limb kickPelvis = request.supportLeg == IndykickRequest::left ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    kickFootPoseInKickPelvis = theRobotModel.limbs[kickPelvis].invert();
  }
  kickFootPoseInKickPelvis.conc(kickFoot.pose);

  referencePointsPosition[0] = kickFootPoseInKickPelvis.translation;
  referencePointsRotation[0] = kickFootPoseInKickPelvis.rotation.getAngleAxis();
  kickFootPoseInKickPelvis.translate(0.0f, 0.0f, p.heightOffset * 2.5f);
  referencePointsPosition[5] = kickFootPoseInKickPelvis.translation; // And end in the same position, but on the ground.
  referencePointsRotation[5] = referencePointsRotation[0];
}

void BSplineGenerator::calculateStrikeoutPointInSupportFoot()
{
  const bool left = request.supportLeg == IndykickRequest::left;
  const float sign = left ? -1.0f : 1.0f;
  Pose3D ballInSupportFoot(RotationMatrix(referencePointsRotation[2]), referencePointsPosition[2]);
  strikeoutCircleRadius = 110.0f;
  const float rSqr = sqr(strikeoutCircleRadius);

  ballPlane1 = Vector2<>(referencePointsPosition[2].x, referencePointsPosition[2].y);
  ballPlane2 = Vector2<>(referencePointsPosition[3].x, referencePointsPosition[3].y);
  ballPlane = (ballPlane1 + ballPlane2) / 2.0f;
  pelvisPlane = Vector2<>(15.0f + 5.0f, sign * 75.0f);
  const Vector2<> directionPlane(kickDirectionInSupportFoot.x, kickDirectionInSupportFoot.y);
  const float dd = directionPlane.squareAbs();
  const float pp = pelvisPlane.squareAbs();
  const float bb = ballPlane.squareAbs();
  const float bp = ballPlane * pelvisPlane;
  const Vector2<> ballPlaneInPelvisPlane = ballPlane - pelvisPlane;
  const float dbpOverDd = directionPlane * ballPlaneInPelvisPlane / dd;
  const float square = (rSqr - bb + 2.0f * bp  - pp) / dd - sqr(dbpOverDd);
  const float root = std::sqrt(square);
  const float lambda1 = -dbpOverDd - root;
  const float lambda2 = -dbpOverDd + root;

  /* Assert that if the ball is to the right of the support foot, then the support foot must be the left foot. */
  ASSERT(ballPlane.y < 0.0f == left);
  strikeOut = ballPlane + directionPlane * lambda1;
  if(square < 0.0f || strikeOut.x > pelvisPlane.x)
  {
    strikeOut.x = pelvisPlane.x;
    strikeOut.y = pelvisPlane.y + sign * strikeoutCircleRadius;
  }
  ASSERT((strikeOut - pelvisPlane).squareAbs() <= rSqr * 1.1f);

  /* Limit distance between ball and strike out reference point. */
  Vector2<> strikeOutInBall = strikeOut - ballPlane1;
  if(strikeOutInBall.squareAbs() > sqr(p.strikeoutBallDistanceLimit))
  {
    strikeOutInBall.normalize(p.strikeoutBallDistanceLimit);
    strikeOut = ballPlane1 + strikeOutInBall;
  }

  const float footSafetyMargin = 5.0f;
  swingOut = ballPlane + directionPlane * lambda2;
  if(square < 0.0f
     || (sign == 1.0f && swingOut.y < pelvisPlane.y + footSafetyMargin)
     || (sign == -1.0f && swingOut.y > pelvisPlane.y - footSafetyMargin))
  {
    swingOut.y = pelvisPlane.y + sign * footSafetyMargin;
    const float scale = (swingOut.y - ballPlane2.y) / directionPlane.y;
    swingOut.x = ballPlane2.x + directionPlane.x * scale;
  }

  /* Limit distance between ball and swing out reference point. */
  Vector2<> swingOutInBall = swingOut - ballPlane2;
  if(swingOutInBall.squareAbs() > sqr(p.swingoutBallDistanceLimit))
  {
    swingOutInBall.normalize(p.swingoutBallDistanceLimit);
    swingOut = ballPlane2 + swingOutInBall;
  }

  referencePointsPosition[1] = Vector3<>(strikeOut.x, strikeOut.y, p.strikeoutHeight);
  referencePointsRotation[1] = Vector3<>(0.0f, 0.0f, 0.0f);
  referencePointsPosition[4] = Vector3<>(swingOut.x, swingOut.y, p.strikeoutHeight);
  referencePointsRotation[4] = Vector3<>(0.0f, 0.0f, 0.0f);
}

void BSplineGenerator::calculateStrikeoutPoint(const Pose3D& supportFootInKickPelvis)
{
  const Pose3D strikeOutKickFootInSupportFoot(referencePointsPosition[1]);
  Pose3D strikeOutKickFootInKickPelvis(supportFootInKickPelvis);
  strikeOutKickFootInKickPelvis.conc(strikeOutKickFootInSupportFoot);
  referencePointsRotation[1] = strikeOutKickFootInKickPelvis.rotation.getAngleAxis();
  referencePointsPosition[1] = strikeOutKickFootInKickPelvis.translation;

  const Pose3D swingOutKickFootInSupportFoot(referencePointsPosition[4]);
  Pose3D swingOutKickFootInKickPelvis(supportFootInKickPelvis);
  swingOutKickFootInKickPelvis.conc(swingOutKickFootInSupportFoot);
  referencePointsRotation[4] = swingOutKickFootInKickPelvis.rotation.getAngleAxis();
  referencePointsPosition[4] = swingOutKickFootInKickPelvis.translation;
}

void BSplineGenerator::generateBallContactReferencePointsInSupportFoot()
{
  const MassCalibration::Limb kickFoot = request.supportLeg == IndykickRequest::left ? MassCalibration::footRight
                                                                                  : MassCalibration::footLeft;
  /* Calculate contact point on the foot contour */
  const RotationMatrix rotRobotInKickFoot = (theTorsoMatrix.rotation * theRobotModel.limbs[kickFoot].rotation).invert();
  Vector3<> direction3DinRobot(request.kickDirection.x,
                               request.kickDirection.y,
                               0.0f);
  direction3DinRobot.normalize();
  const Vector3<> direction3DinKickFoot = rotRobotInKickFoot * direction3DinRobot;
  Vector2<> directionInKickFoot(direction3DinKickFoot.x,
                                direction3DinKickFoot.y);
  const float directionAngle = directionInKickFoot.angle();
  const float footRotationAngle = p.footRotationFactor * directionAngle;
  directionInKickFoot.rotate(-footRotationAngle);
  Vector2<> tangent2D;
  const Vector2<> contactPoint2D = footFrontContour.calcContactPoint(directionInKickFoot,
                                                                     request.supportLeg == IndykickRequest::right,
                                                                     tangent2D);
  contactPoint.x = contactPoint2D.x;
  contactPoint.y = contactPoint2D.y;
  contactPoint.z = -theRobotDimensions.heightLeg5Joint;
  contactTangent.x = tangent2D.x;
  contactTangent.y = tangent2D.y;
  contactTangent.z = 0.0f;
  /* Calculate contact point on the ball contour. */
  const Vector3<> ballContactInSupportFoot1 = this->ballPositionInSupportFoot - this->kickDirectionInSupportFoot * circleOfTheBallRadius * p.circleOffset1;
  const Vector3<> ballContactInSupportFoot2 = this->ballPositionInSupportFoot + this->kickDirectionInSupportFoot * circleOfTheBallRadius * p.circleOffset2;
  Pose3D kickFootPose = Pose3D(ballContactInSupportFoot1);
  kickFootPose.rotateZ(footRotationAngle);
  kickFootPose.translate(-contactPoint);
  referencePointsPosition[2] = kickFootPose.translation;
  referencePointsRotation[2] = kickFootPose.rotation.getAngleAxis();

  kickFootPose = Pose3D(ballContactInSupportFoot2);
  kickFootPose.rotateZ(footRotationAngle);
  kickFootPose.translate(-contactPoint);
  referencePointsPosition[3] = kickFootPose.translation;
  referencePointsRotation[3] = kickFootPose.rotation.getAngleAxis();
}

void BSplineGenerator::generateBallContactReferencePoints(const Pose3D& supportFootInKickPelvis)
{
  /* Convert the already calculated poses into kick pelvis coordinates. */
  const Pose3D beginContact(RotationMatrix(referencePointsRotation[2]), referencePointsPosition[2]);
  const Pose3D endContact(RotationMatrix(referencePointsRotation[3]), referencePointsPosition[3]);

  Pose3D kickFootInKickPelvis(supportFootInKickPelvis);
  kickFootInKickPelvis.conc(beginContact);
  referencePointsPosition[2] = kickFootInKickPelvis.translation;
  referencePointsRotation[2] = kickFootInKickPelvis.rotation.getAngleAxis();
  kickFootInKickPelvis = supportFootInKickPelvis;
  kickFootInKickPelvis.conc(endContact);
  referencePointsPosition[3] = kickFootInKickPelvis.translation;
  referencePointsRotation[3] = kickFootInKickPelvis.rotation.getAngleAxis();
}

void BSplineGenerator::calculateDeBoorMatrixFromDeltas()
{
  /* Calculate the coefficients. */
  for(int i = 0; i < numReferencePoints; ++i)
  {
    alphas[i] = sqr(delta(i)) / (delta(i-2) + delta(i-1) + delta(i));
    betas[i] = delta(i) * (delta(i-2) + delta(i-1)) / (delta(i-2) + delta(i-1) + delta(i))
    + delta(i-1) * (delta(i) + delta(i+1)) / (delta(i-1) + delta(i) + delta(i+1));
    gammas[i] = sqr(delta(i-1)) / (delta(i-1) + delta(i) + delta(i+1));
    omegas[i] = delta(i-1) + delta(i);
  }

  /* Generate matrix to calculate de boor points from the reference points using the deltas. */
  deBoorMatrix[0][0] = betas[1];
  deBoorMatrix[1][0] = gammas[1];
  deBoorMatrix[numReferencePoints - 4][numReferencePoints - 3] = alphas[numReferencePoints - 2];
  deBoorMatrix[numReferencePoints - 3][numReferencePoints - 3] = betas[numReferencePoints - 2];
  for(int i = 2; i < numReferencePoints - 2; ++i)
  {
    deBoorMatrix[i - 2][i - 1] = alphas[i];
    deBoorMatrix[i - 1][i - 1] = betas[i];
    deBoorMatrix[i - 0][i - 1] = gammas[i];
  }
}

void BSplineGenerator::updateBallContactPhaseDelta()
{
  generateDeltasAlt();
  generateDeBoorPointsAlt();
  generateDeltasWithLengths();
  generateDeBoorPointsAlt();
}

void BSplineGenerator::generateDeltasAlt()
{
  const float stepDownSpeed = this->kickSpeed / 4.0f;
  deltas[2] = circleOfTheBallRadius * (p.circleOffset1 + p.circleOffset2) / this->kickSpeed;
  deltas[numDeltas - 1] = (referencePointsPosition[numDeltas] - referencePointsPosition[numDeltas - 1]).abs() / stepDownSpeed; // This is nearly a line.
  if(deltas[numDeltas - 1] > kickDuration / 3.0f)
    deltas[numDeltas - 1] = kickDuration / 3.0f;
  const float duration = kickDuration - deltas[2] - deltas[numDeltas - 1];
  float sum = 0.0f;
  for(int i = 0; i < numDeltas - 1; ++i)
    if(i != 2)
    {
      deltas[i] = std::sqrt((referencePointsPosition[i + 1] - referencePointsPosition[i]).abs());
      sum += deltas[i];
    }
  const float normalizer = duration / sum;
  for(int i = 0; i < numDeltas - 1; ++i)
    if(i != 2)
      deltas[i] *= normalizer;

  calculateDeBoorMatrixFromDeltas();
}

void BSplineGenerator::generateDeltasWithLengths()
{
  /* This calculation ignores segment 2 (the ball-contact-segment), as the delta for this
   * segment is fixed by the demanded kick speed. */
  float segmentLengths[numDeltas];
  float splineLength = 0.0f;
  for(int i = 0; i < numDeltas - 1; ++i)
  {
    segmentLengths[i] = calculateLengthOfSegment(i);
    if(i != 2)
      splineLength += segmentLengths[i];
  }
  deltas[2] = circleOfTheBallRadius * (p.circleOffset1 + p.circleOffset2) / this->kickSpeed;

  float segmentSpeed[numDeltas];
  for(int i = 0; i < numDeltas; ++i)
    segmentSpeed[i] = segmentLengths[i] / deltas[i];

  const float duration = kickDuration - deltas[2] - deltas[numDeltas - 1];
  const float normalizer = duration / splineLength;
  for(int i = 0; i < numDeltas - 1; ++i)
    if(i != 2)
      deltas[i] = segmentLengths[i] * normalizer;

  calculateDeBoorMatrixFromDeltas();
}

void BSplineGenerator::generateDeBoorPointsAlt()
{
    /* Build the reference vectors (from the reference points). */
  Vector3<> referenceVectorPosition[numReferencePoints - 2];
  Vector3<> referenceVectorRotation[numReferencePoints - 2];
  referenceVectorPosition[0] = referencePointsPosition[1] * omegas[1] - referencePointsPosition[0] * alphas[1];
  referenceVectorPosition[numReferencePoints - 3] = referencePointsPosition[numReferencePoints - 2] * omegas[numReferencePoints - 2]
    - referencePointsPosition[numReferencePoints - 1] * gammas[numReferencePoints - 2];
  referenceVectorRotation[0] = referencePointsRotation[1] * omegas[1] - referencePointsRotation[0] * alphas[1];
  referenceVectorRotation[numReferencePoints - 3] = referencePointsRotation[numReferencePoints - 2] * omegas[numReferencePoints - 2]
    - referencePointsRotation[numReferencePoints - 1] * gammas[numReferencePoints - 2];
  for(int i = 1; i < numReferencePoints - 3; ++i)
  {
    referenceVectorPosition[i] = referencePointsPosition[i + 1] * omegas[i + 1];
    referenceVectorRotation[i] = referencePointsRotation[i + 1] * omegas[i + 1];
  }

  const Matrix4x4f deBoorMatrixInv = deBoorMatrix.invert();
  for(int i = 0; i < numReferencePoints - 2; ++i)
  {
    deBoorPointsPosition[i] = referenceVectorPosition[0] * deBoorMatrixInv[0][i];
    deBoorPointsRotation[i] = referenceVectorRotation[0] * deBoorMatrixInv[0][i];
    for(int j = 1; j < numReferencePoints - 2; ++j)
    {
      deBoorPointsPosition[i] += referenceVectorPosition[j] * deBoorMatrixInv[j][i];
      deBoorPointsRotation[i] += referenceVectorRotation[j] * deBoorMatrixInv[j][i];
    }
  }
}

bool BSplineGenerator::refineSpeedDS()
{
  ASSERT(kickSpeed > 0.0f);
  const float radius = theRobotDimensions.upperLegLength + theRobotDimensions.lowerLegLength;
  const float radiusSqr = sqr(0.95f * radius);
  bool feasible = true;
  for(int i = 0; i < numReferencePoints -2; ++i)
    if(deBoorPointsPosition[i].squareAbs() > radiusSqr)
      feasible = false;
  if(feasible)
    return false; /* If the spline is ok, don't mess around with it. */

  /* Damn, the spline may be infeasible. So let's check the maximum distance of the curve from the pelvis. */
  Vector3<> point;
  const float localKnotMax = calculateMaximumOfSegment(1, point);

  float maxLengthSqr = point.squareAbs();
  if(maxLengthSqr <= radiusSqr)
    return false; /* The maximum length of the curve is lower than the leg length, so this spline is also ok. */

  float speed[2] = {this->kickSpeed, 4.0f / 5.0f * this->kickSpeed};
  float error[2] = {std::abs(maxLengthSqr - radiusSqr), -1.0f};
  kickSpeed = speed[1];

  updateBallContactPhaseDelta();
  generateDeBoorPointsAlt();
  evaluateSegment(1, localKnotMax, point);
  maxLengthSqr = point.squareAbs();
  error[1] = std::abs(maxLengthSqr - radiusSqr);

  while(true)
  {
    /* Sort */
    if(error[1] < error[0])
    {
      float t = error[0];
      error[0] = error[1];
      error[1] = t;
      t = speed[0];
      speed[0] = speed[1];
      speed[1] = t;
    }

    if(std::abs(error[1] - error[0]) < 5.0f)
      return false;

    /* Reflection */
    float errorNew;
    kickSpeed = 2.0f * speed[0] - speed[1];
    if(kickSpeed < 0.5f)
    {
      errorNew = 1e9f + (0.5f - kickSpeed) * 1e9f;
    }
    else
    {
      updateBallContactPhaseDelta();
      generateDeBoorPointsAlt();
      evaluateSegment(1, localKnotMax, point);
      maxLengthSqr = point.squareAbs();
      errorNew = std::abs(maxLengthSqr - radiusSqr);
    }
    if(errorNew <= error[1])
    {
      error[1] = errorNew;
      speed[1] = kickSpeed;
    }
    if(errorNew <= error[0])
    {
      /* Expand */
      kickSpeed = -speed[0] + 2.0f * speed[1];
      if(kickSpeed < 0.5f)
      {
        errorNew = 1e9f + (0.5f - kickSpeed) * 1e9f;
      }
      else
      {
        updateBallContactPhaseDelta();
        generateDeBoorPointsAlt();
        evaluateSegment(1, localKnotMax, point);
        maxLengthSqr = point.squareAbs();
        errorNew = std::abs(maxLengthSqr - radiusSqr);
      }
      if(errorNew < error[1])
      {
        error[1] = errorNew;
        speed[1] = kickSpeed;
      }
    }
    else // if(errorNew >= error[0])
    {
      /* Contract */
      kickSpeed = (speed[0] + speed[1]) / 2.0f;
      if(kickSpeed < 0.5f)
      {
        errorNew = 1e9f + (0.5f - kickSpeed) * 1e9f;
      }
      else
      {
        updateBallContactPhaseDelta();
        generateDeBoorPointsAlt();
        evaluateSegment(1, localKnotMax, point);
        maxLengthSqr = point.squareAbs();
        errorNew = std::abs(maxLengthSqr - radiusSqr);
      }
      if(errorNew <= error[1])
      {
        error[1] = errorNew;
        speed[1] = kickSpeed;
      }
      else
      {
        /* Contract again */
        kickSpeed = (speed[0] + speed[1]) / 2.0f;
        if(kickSpeed < 0.5f)
        {
          errorNew = 1e9f + (0.5f - kickSpeed) * 1e9f;
        }
        else
        {
          updateBallContactPhaseDelta();
          generateDeBoorPointsAlt();
          evaluateSegment(1, localKnotMax, point);
          maxLengthSqr = point.squareAbs();
          errorNew = std::abs(maxLengthSqr - radiusSqr);
        }
        speed[1] = kickSpeed;
        error[1] = errorNew;
      }
    }
  }
}

void BSplineGenerator::generateBezierPoints()
{
  const MassCalibration::Limb supportFoot = request.supportLeg == IndykickRequest::left
    ? MassCalibration::footLeft : MassCalibration::footRight;
  const MassCalibration::Limb kickPelvis = request.supportLeg == IndykickRequest::left
    ? MassCalibration::pelvisRight : MassCalibration::pelvisLeft;

  Pose3D kickPelvisInReferenceSystem;
  if(referenceCoordinateSystem == KickFootPose::supportFoot)
    kickPelvisInReferenceSystem = theRobotModel.limbs[supportFoot].invert();
  kickPelvisInReferenceSystem.conc(theRobotModel.limbs[kickPelvis]);

  const bool rotation = true;
  const bool position = false;
  for(int i = 0; i < numReferencePoints - 1; ++i)
  {
    /* The bezier points that coincide with the reference points. */
    bezierPointsPosition[3 * i] = kickPelvisInReferenceSystem * referencePointsPosition[i];
    bezierPointsRotation[3 * i] = (kickPelvisInReferenceSystem.rotation * RotationMatrix(referencePointsRotation[i])).getAngleAxis();

    const float totalDelta = delta(i - 1) + delta(i) + delta(i + 1);
    bezierPointsPosition[3 * i + 1] = deBoor(i, position) * (delta(i) + delta(i+1)) / totalDelta
                                    + deBoor(i + 1, position) * delta(i-1) / totalDelta;
    bezierPointsPosition[3 * i + 1] = kickPelvisInReferenceSystem * bezierPointsPosition[3 * i + 1];

    bezierPointsPosition[3 * i + 2] = deBoor(i, position) * delta(i+1) / totalDelta
                                    + deBoor(i + 1, position) * (delta(i-1) + delta(i)) / totalDelta;
    bezierPointsPosition[3 * i + 2] = kickPelvisInReferenceSystem * bezierPointsPosition[3 * i + 2];

    bezierPointsRotation[3 * i + 1] = deBoor(i, rotation) * (delta(i) + delta(i+1)) / totalDelta
                                    + deBoor(i + 1, rotation) * delta(i-1) / totalDelta;
    bezierPointsRotation[3 * i + 1] = (kickPelvisInReferenceSystem.rotation * RotationMatrix(bezierPointsRotation[3 * i + 1])).getAngleAxis();

    bezierPointsRotation[3 * i + 2] = deBoor(i, rotation) * delta(i+1) / totalDelta
                                    + deBoor(i + 1, rotation) * (delta(i-1) + delta(i)) / totalDelta;
    bezierPointsRotation[3 * i + 2] = (kickPelvisInReferenceSystem.rotation * RotationMatrix(bezierPointsRotation[3 * i + 2])).getAngleAxis();
  }
  bezierPointsPosition[3 * (numReferencePoints - 1)] = kickPelvisInReferenceSystem * referencePointsPosition[numReferencePoints - 1];
  bezierPointsRotation[3 * (numReferencePoints - 1)] = (kickPelvisInReferenceSystem.rotation * RotationMatrix(referencePointsRotation[numReferencePoints - 1])).getAngleAxis();
}

float BSplineGenerator::delta(int i) const
{
  if(0 <= i && i < numDeltas)
    return deltas[i];
  return 0.0f;
}

Vector3<>& BSplineGenerator::deBoor(const int i, const bool rotation)
{
  ASSERT(i >= -1);
  ASSERT(i <= numReferencePoints);
  if(i <= 0)
    return rotation ? referencePointsRotation[0] : referencePointsPosition[0];
  else if (i >= numReferencePoints - 1)
    return rotation ? referencePointsRotation[numReferencePoints - 1] : referencePointsPosition[numReferencePoints - 1];
  else
    return rotation ? deBoorPointsRotation[i - 1] : deBoorPointsPosition[i - 1];
}

BezierCurve BSplineGenerator::getPositions(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const
{
  ASSERT(phaseId >= p.firstBSplinePhase);
  const int i = phaseId - p.firstBSplinePhase;
  referenceCoordinateSystem = this->referenceCoordinateSystem;
  return BezierCurve(bezierPointsPosition[3 * i + 0], bezierPointsPosition[3 * i + 1], bezierPointsPosition[3 * i + 2], bezierPointsPosition[3 * i + 3]);
}

AngleBezierCurve BSplineGenerator::getAngles(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const
{
  ASSERT(phaseId >= p.firstBSplinePhase);
  const int i = phaseId - p.firstBSplinePhase;
  referenceCoordinateSystem = this->referenceCoordinateSystem;
  return AngleBezierCurve(bezierPointsRotation[3 * i + 0], bezierPointsRotation[3 * i + 1], bezierPointsRotation[3 * i + 2], bezierPointsRotation[3 * i + 3]);
}

BezierCurveReference BSplineGenerator::getPositionsReference(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const
{
  ASSERT(phaseId >= p.firstBSplinePhase);
  const int i = phaseId - p.firstBSplinePhase;
  referenceCoordinateSystem = this->referenceCoordinateSystem;
  return BezierCurveReference(bezierPointsPosition[3 * i + 0], bezierPointsPosition[3 * i + 1], bezierPointsPosition[3 * i + 2], bezierPointsPosition[3 * i + 3]);
}

AngleBezierCurveReference BSplineGenerator::getAnglesReference(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const
{
  ASSERT(phaseId >= p.firstBSplinePhase);
  const int i = phaseId - p.firstBSplinePhase;
  referenceCoordinateSystem = this->referenceCoordinateSystem;
  return AngleBezierCurveReference(bezierPointsRotation[3 * i + 0], bezierPointsRotation[3 * i + 1], bezierPointsRotation[3 * i + 2], bezierPointsRotation[3 * i + 3]);
}

void BSplineGenerator::drawDeBoorPoints()
{
  for(int i = -1; i <= numReferencePoints; ++i)
  {
    const Vector3<>& d = deBoor(i, false);
    SPHERE3D("module:IndykickEngine:deboorLeft", d.x, d.y, d.z, 5, ColorClasses::red);
    SPHERE3D("module:IndykickEngine:deboorRight", d.x, d.y, d.z, 5, ColorClasses::red);
  }
  drawSpline();
}

void BSplineGenerator::performDeBoorAlgorithm(Vector3<> intermediates[4][4], const int segment, const float uStart, const float localKnot)
{
  const float u = uStart + localKnot;
  for(int k = 1; k < 4; ++k)
    for(int i = 0; i < 4 - k; ++i)
    {
      float ulow = 0.0f;
      for(int m = 0; m < segment - (4-1) + k + i; ++m)
        ulow += delta(m);
      float uhigh = 0.0f;
      for(int m = 0; m < segment + i + 1; ++m)
        uhigh += delta(m);

      float a = (u - ulow) / (uhigh - ulow);

      intermediates[k][i] = intermediates[k-1][i] * (1.0f - a) + intermediates[k-1][i+1] * a;
    }
}

bool BSplineGenerator::calculateMaximumOfSubsegment(Vector3<> intermediates[4][4], const int segment, const float segmentUStart,
                                                     float uMin, float uMax, float valueMin, float valueMax, float& uMaximum, Vector3<>& maximum)
{
  Vector3<> pointDerivative;
  float value;
  bool outOfRange = false;
  bool failure = false;
  float u = uMin - ((uMax - uMin) * valueMin) / (valueMax - valueMin);
  while(true)
  {
    outOfRange = (u < uMin) || (u > uMax);
    if(outOfRange)
    {
      failure = true;
      break;
    }
    performDeBoorAlgorithm(intermediates, segment, segmentUStart, u);
    pointDerivative = intermediates[2][1] - intermediates[2][0];
    value = intermediates[3][0] * pointDerivative;
    if(std::abs(value) < 5.0f)
    {
      maximum = intermediates[3][0];
      uMaximum = u;
      return true;
    }
    if((value < 0.0f) == (valueMin < 0.0f))
    {
      valueMin = value;
      uMin = u;
    }
    else if((value < 0.0f) == (valueMax < 0.0f))
    {
      valueMax = value;
      uMax = u;
    }
    else
    {
      failure = true;
      break;
    }
    u = uMin - ((uMax - uMin) * valueMin) / (valueMax - valueMin);
  }

  if(failure)
  {
    /* Handle multiple or no zeros. */
    if(uMax < uMin)
    {
      const float t = uMin;
      uMin = uMax;
      uMax = t;
    }
    if(uMax - uMin < 1.0f)
      return false; /* This segment does not have a zero. */

    /* This segment spans probably mutliple zeros. */
    if(outOfRange)
    {
      u = (uMax + uMin) / 2.0f;
      performDeBoorAlgorithm(intermediates, segment, segmentUStart, u);
      pointDerivative = intermediates[2][1] - intermediates[2][0];
      value = intermediates[3][0] * pointDerivative;
    }
    float uMax1, uMax2;
    Vector3<> max1, max2;
    const bool hasMax1 = calculateMaximumOfSubsegment(intermediates, segment, segmentUStart, uMin, u, valueMin, value, uMax1, max1);
    const bool hasMax2 = calculateMaximumOfSubsegment(intermediates, segment, segmentUStart, u, uMax, value, valueMax, uMax2, max2);

    if(!(hasMax1 || hasMax2))
      return false;
    if(!hasMax1)
    {
      uMaximum = uMax2;
      maximum = max2;
    }
    else if(!hasMax2)
    {
      uMaximum = uMax1;
      maximum = max1;
    }
    else if(max1.squareAbs() > max2.squareAbs())
    {
      uMaximum = uMax1;
      maximum = max1;
    }
    else
    {
      uMaximum = uMax2;
      maximum = max2;
    }
  }
  return true;
}

float BSplineGenerator::calculateMaximumOfSegment(const int segment, Vector3<>& maximum)
{
  Vector3<> intermediates[4][4];
  float uStart = 0.0f;
  for(int j = 0; j < segment; ++j)
    uStart += delta(j);
  for(int i = 0; i < 4; ++i)
    intermediates[0][i] = deBoor(segment + i - 1, false);

  float uMin = 0.1f * delta(segment);
  float uMax = 0.9f * delta(segment);
  performDeBoorAlgorithm(intermediates, segment, uStart, uMin);
  Vector3<> pointDerivative = intermediates[2][1] - intermediates[2][0];
  float derivativeMin = intermediates[3][0] * pointDerivative;
  performDeBoorAlgorithm(intermediates, segment, uStart, uMax);
  pointDerivative = intermediates[2][1] - intermediates[2][0];
  float derivativeMax = intermediates[3][0] * pointDerivative;

  float u;
  calculateMaximumOfSubsegment(intermediates, segment, uStart, uMin, uMax, derivativeMin, derivativeMax, u, maximum);
  return u;
}

void BSplineGenerator::evaluateSegment(const int segment, const float localKnot, Vector3<> &point)
{
  Vector3<> intermediates[4][4];
  float uStart = 0.0f;
  for(int j = 0; j < segment; ++j)
    uStart += delta(j);
  for(int i = 0; i < 4; ++i)
    intermediates[0][i] = deBoor(segment + i - 1, false);
  performDeBoorAlgorithm(intermediates, segment, uStart, localKnot);
  point = intermediates[3][0];
}

void BSplineGenerator::drawMaximumOfSegment(const int segment)
{
  Vector3<> max;
  calculateMaximumOfSegment(1, max);
  SPHERE3D("module:IndykickEngine:segmentMax", max.x, max.y, max.z, 7, ColorClasses::blue);
}

void BSplineGenerator::drawPointOnSplineWithTangent(const int segment, const float knotRatio)
{
  Vector3<> intermediates[4][4];
  float uStart = 0.0f;
  for(int j = 0; j < segment; ++j)
    uStart += delta(j);
  for(int i = 0; i < 4; ++i)
    intermediates[0][i] = deBoor(segment + i - 1, false);

  performDeBoorAlgorithm(intermediates, segment, uStart, delta(segment) * knotRatio);

  const Vector3<>& d = intermediates[3][0];
  const Vector3<> tangent = (intermediates[2][1] - intermediates[2][0]);
  const Vector3<> d1 = d + tangent;

  Vector3<> derivative2 = (intermediates[1][2] / (delta(segment) + delta(segment + 1))
    - intermediates[1][1] * (1.0f / (delta(segment) + delta(segment + 1)) + 1.0f / (delta(segment - 1) + delta(segment)))
    + intermediates[1][0] / (delta(segment - 1) + delta(segment))) * 6.0f / delta(segment);
  derivative2.normalize(tangent.abs() / 2.0f);
  const Vector3<> d2 = d + derivative2;

  SPHERE3D("module:IndykickEngine:secondsegment", d.x, d.y, d.z, 7, ColorClasses::blue);
  LINE3D("module:IndykickEngine:secondsegment", d.x, d.y, d.z, d1.x, d1.y, d1.z, 5, ColorClasses::red);
  LINE3D("module:IndykickEngine:secondsegment", d.x, d.y, d.z, d2.x, d2.y, d2.z, 5, ColorClasses::green);
}

float BSplineGenerator::calculateLengthOfSegment(const int j)
{
  ASSERT(j >= 0);
  ASSERT(j < numDeltas);
  const int numSamples = 10;
  const float step = delta(j) / static_cast<float>(numSamples);

  float uj = 0.0f;
  for(int i = 0; i < j; ++i)
    uj += delta(i);

  Vector3<> intermediates[4][4];
  for(int i = 0; i < 4; ++i)
    intermediates[0][i] = deBoor(j + i - 1, false);

  performDeBoorAlgorithm(intermediates, j, uj, 0.0f);
  Vector3<> lastPoint = intermediates[3][0];

  float length = 0.0f;
  for(int s = 1; s < numSamples; ++s)
  {
    const float localKnot = s * step;
    performDeBoorAlgorithm(intermediates, j, uj, localKnot);
    const Vector3<>& d = intermediates[3][0];
    const Vector3<> diff = d - lastPoint;
    length += diff.abs();

    lastPoint = d;
  }

  return length;
}

void BSplineGenerator::drawSegment(int j)
{
  ASSERT(j >= 0);
  ASSERT(j < numDeltas);
  const float legLengthSqr = sqr(theRobotDimensions.upperLegLength + theRobotDimensions.lowerLegLength);
  const int numSamples = 100;
  const float step = delta(j) / static_cast<float>(numSamples);

  float uj = 0.0f;
  for(int i = 0; i < j; ++i)
    uj += delta(i);

  Vector3<> intermediates[4][4];
  for(int i = 0; i < 4; ++i)
    intermediates[0][i] = deBoor(j + i - 1, false);

  for(int s = 1; s < numSamples; ++s)
  {
    performDeBoorAlgorithm(intermediates, j, uj, s * step);
    const Vector3<>& d = intermediates[3][0];

    const ColorClasses::Color color = d.squareAbs() < legLengthSqr ? ColorClasses::yellow : ColorClasses::red;
    SPHERE3D("module:IndykickEngine:deboorRight", d.x, d.y, d.z, 3, color);
    SPHERE3D("module:IndykickEngine:deboorLeft", d.x, d.y, d.z, 3, color);
  }
}

void BSplineGenerator::drawSpline()
{
  for(int i = 0; i < numDeltas; ++i)
    drawSegment(i);
//  drawSegment(3);
}

void BSplineGenerator::drawReferencePoints()
{
  for(int i = 0; i < numReferencePoints; ++i)
  {
    const Vector3<>& p = referencePointsPosition[i];
    SPHERE3D("module:IndykickEngine:curveReference", p.x, p.y, p.z, 5, ColorClasses::blue);
  }
}

void BSplineGenerator::drawStrikeOutBall(const bool left) const
{
  if(left)
  {
    CYLINDER3D("module:IndykickEngine:strikeOutBallLeft", pelvisPlane.x, pelvisPlane.y, p.strikeoutHeight, 0.0f, 0.0f, 0.0f,
               strikeoutCircleRadius, 1.0f, ColorClasses::red);
    POINT3D("module:IndykickEngine:strikeOutBallLeft", strikeOut.x, strikeOut.y, p.strikeoutHeight, 6.0f, ColorClasses::blue);
    POINT3D("module:IndykickEngine:strikeOutBallLeft", swingOut.x, swingOut.y, p.strikeoutHeight, 6.0f, ColorClasses::blue);
    POINT3D("module:IndykickEngine:strikeOutBallLeft", ballPlane.x, ballPlane.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
    POINT3D("module:IndykickEngine:strikeOutBallLeft", ballPlane1.x, ballPlane1.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
    POINT3D("module:IndykickEngine:strikeOutBallLeft", ballPlane2.x, ballPlane2.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
  }
  else
  {
    CYLINDER3D("module:IndykickEngine:strikeOutBallRight", pelvisPlane.x, pelvisPlane.y, p.strikeoutHeight, 0.0f, 0.0f, 0.0f,
               strikeoutCircleRadius, 1.0f, ColorClasses::red);
    POINT3D("module:IndykickEngine:strikeOutBallRight", strikeOut.x, strikeOut.y, p.strikeoutHeight, 6.0f, ColorClasses::blue);
    POINT3D("module:IndykickEngine:strikeOutBallRight", swingOut.x, swingOut.y, p.strikeoutHeight, 6.0f, ColorClasses::blue);
    POINT3D("module:IndykickEngine:strikeOutBallRight", ballPlane.x, ballPlane.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
    POINT3D("module:IndykickEngine:strikeOutBallRight", ballPlane1.x, ballPlane1.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
    POINT3D("module:IndykickEngine:strikeOutBallRight", ballPlane2.x, ballPlane2.y, p.strikeoutHeight, 6.0f, ColorClasses::yellow);
  }
}

bool BSplineGenerator::modifiesDuration() const
{
  return durations[p.firstBSplinePhase] >= 0;
}

int BSplineGenerator::getDuration(const BKickPhase::PhaseId phaseId) const
{
  ASSERT(phaseId >= p.firstBSplinePhase);
  ASSERT(phaseId < BKickPhase::numOfPhaseIds);
  return durations[phaseId];
}

float BSplineGenerator::getDelta(const BKickPhase::PhaseId phaseId) const
{
  const int deltaIdx = phaseId - p.firstBSplinePhase;
  ASSERT(deltaIdx >= 0);
  return delta(deltaIdx);
}

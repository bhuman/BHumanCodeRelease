/**
* @file UKFSample.cpp
*
* Implementation of Unscented Kalman Filter for robot pose estimation
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/


#include "UKFSample.h"
#include "FieldModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Vector3.h"

using namespace std;

void UKFSample::init(const Pose2D& pose, const SelfLocatorParameters& parameters)
{
  mean.x = pose.translation.x;
  mean.y = pose.translation.y;
  mean.z = pose.rotation;
  cov = Matrix3x3f();
  cov[0][0] = sqr(parameters.defaultPoseDeviation.translation.x);
  cov[1][1] = sqr(parameters.defaultPoseDeviation.translation.y);
  cov[2][2] = sqr(parameters.defaultPoseDeviation.rotation);
  mirrored = false;
  // Buffer is always initially filled with average value
  for(int i=0; i<validityBuffer.getMaxEntries(); ++i)
    validityBuffer.add(0.5f);
}


void UKFSample::motionUpdate(const Pose2D& odometryOffset, const SelfLocatorParameters& parameters)
{
  generateSigmaPoints();
  performOdometryUpdate(odometryOffset, parameters);
}


Pose2D UKFSample::getPose() const
{
  return Pose2D(mean.z, mean.x, mean.y);
}


void UKFSample::mirror()
{
  const Pose2D newPose = Pose2D(pi) + getPose();
  mean.x = newPose.translation.x;
  mean.y = newPose.translation.y;
  mean.z = newPose.rotation;
}


float UKFSample::computeValidity(const FieldDimensions& fieldDimensions)
{
  const Vector2<> pos = getPose().translation;
  if(fieldDimensions.isInsideCarpet(pos))
    return validity = static_cast<float>(validityBuffer.getSum()) / validityBuffer.getNumberOfEntries();
  else
    return validity = 0.f;
}


void UKFSample::invalidate()
{
  for(int i=0; i<validityBuffer.getMaxEntries(); ++i)
    validityBuffer.add(0.f);
}


float UKFSample::getVarianceWeighting() const
{
  return std::max(cov[0].x, cov[1].y) * cov[2].z;
}


void UKFSample::twist()
{
  mean.z += pi;
  mean.z = normalize(mean.z);
}


void UKFSample::generateSigmaPoints()
{
  // Cholesky decomposition
  const float a11 = cov.c[0].x;
  const float a21 = (cov.c[0].y + cov.c[1].x) * 0.5f;
  const float a31 = (cov.c[0].z + cov.c[2].x) * 0.5f;
  const float a22 = cov.c[1].y;
  const float a32 = (cov.c[1].z + cov.c[2].y) * 0.5f;
  const float a33 = cov.c[2].z;

  float& l11(l.c[0].x);
  float& l21(l.c[0].y);
  float& l31(l.c[0].z);
  float& l22(l.c[1].y);
  float& l32(l.c[1].z);
  float& l33(l.c[2].z);

  //ASSERT(a11 >= 0.f);
  l11 = sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;
  l31 = a31 / l11;
  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
  l32 = (a32 - l31 * l21) / l22;
  //ASSERT(a33 - l31 * l31 - l32 * l32 >= 0.f);
  l33 = sqrt(std::max<>(a33 - l31 * l31 - l32 * l32, 0.f));

  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.c[0];
  sigmaPoints[2] = mean - l.c[0];
  sigmaPoints[3] = mean + l.c[1];
  sigmaPoints[4] = mean - l.c[1];
  sigmaPoints[5] = mean + l.c[2];
  sigmaPoints[6] = mean - l.c[2];
}


void UKFSample::performOdometryUpdate(const Pose2D& odometryOffset, const SelfLocatorParameters& parameters)
{
  // addOdometryToSigmaPoints
  for(int i = 0; i < 7; ++i)
  {
    Vector2<> odo(odometryOffset.translation);
    odo.rotate(sigmaPoints[i].z);
    sigmaPoints[i] += Vector3f(odo.x, odo.y, odometryOffset.rotation);
  }

  // computeMeanOfSigmaPoints
  mean = sigmaPoints[0];
  for(int i = 1; i < 7; ++i)
    mean += sigmaPoints[i];
  mean *= 1.f / 7.f;

  // computeCovOfSigmaPoints
  Vector3f d = sigmaPoints[0] - mean;
  cov = Matrix3x3f(d * d.x, d * d.y, d * d.z);
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = sigmaPoints[i] - mean;
    cov += Matrix3x3f(d * d.x, d * d.y, d * d.z);
  }
  cov *= 0.5f;

  // addProcessNoise
  cov[0][0] += sqr(parameters.filterProcessDeviation.translation.x);
  cov[1][1] += sqr(parameters.filterProcessDeviation.translation.y);
  cov[2][2] += sqr(parameters.filterProcessDeviation.rotation);

  Vector2<> odo(odometryOffset.translation);
  odo.rotate(mean.z);
  cov[0][0] += sqr(odo.x * parameters.odometryDeviation.translation.x);
  cov[1][1] += sqr(odo.y * parameters.odometryDeviation.translation.y);
  cov[2][2] += sqr(odometryOffset.rotation * parameters.odometryDeviation.rotation);
  cov[2][2] += sqr(odo.x * parameters.odometryRotationDeviation.x);
  cov[2][2] += sqr(odo.y * parameters.odometryRotationDeviation.y);

  mean.z = normalize(mean.z);
}


void UKFSample::updateByGoalPercept(const GoalPercept& goalPercept, const FieldModel& fieldModel, const SelfLocatorParameters& parameters,
                                    const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
  const Pose2D robotPose = getPose();
  for(vector<GoalPost>::const_iterator goalPost = goalPercept.goalPosts.begin(); goalPost != goalPercept.goalPosts.end(); goalPost++)
  {
    // Data association:
    Vector2<> associatedPost;
    bool postFound(false);
    if(goalPost->position == GoalPost::IS_UNKNOWN)
      postFound = fieldModel.getAssociatedUnknownGoalPost(robotPose, goalPost->positionOnField, associatedPost);
    else // goalPost->IS_LEFT || goalPost->IS_RIGHT
      postFound = fieldModel.getAssociatedKnownGoalPost(robotPose, goalPost->positionOnField, goalPost->position == GoalPost::IS_LEFT, associatedPost);

    // Filter update:
    if(postFound)
    {
      Matrix2x2f cov = getCovOfPointInWorld(goalPost->positionOnField, 0.f, motionInfo, cameraMatrix, parameters);
      landmarkSensorUpdate(associatedPost, Vector2f(goalPost->positionOnField.x, goalPost->positionOnField.y), cov);
      validityBuffer.add(1.f);
    }
    else
    {
      validityBuffer.add(0.f);
    }
  }
}


void UKFSample::updateByLinePercept(const LinePercept& linePercept, const FieldModel& fieldModel, const SelfLocatorParameters& parameters,
                                    const FieldDimensions& fieldDimensions, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
  // Check for center circle:
  bool centerCircleCanBeUsed = false;
  if(linePercept.circle.found)
  {
    const Pose2D robotPose = getPose();
    const Vector2<> circleInWorld = robotPose * linePercept.circle.pos;
    centerCircleCanBeUsed = circleInWorld.abs() <= parameters.centerCircleAssociationDistance;
    validityBuffer.add(centerCircleCanBeUsed ? 1.f : 0.f);
  }
  // Iterate over all observed lines:
  for(std::vector<LinePercept::Line>::const_iterator it = linePercept.lines.begin(); it != linePercept.lines.end(); ++it)
  {
    const Pose2D robotPose = getPose();
    int index = fieldModel.getIndexOfAssociatedLine(robotPose, it->first, it->last);
    if(index != -1)
    {
      const FieldModel::FieldLine& fieldLine = fieldModel.fieldLines[index];
      Vector2<> center = (it->first + it->last) * 0.5f;
      Vector2<> dir = it->last - it->first;
      dir.normalize();
      Matrix2x2f cov = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix, parameters);
      Vector2<> orthogonalProjectiona = getOrthogonalProjection(it->first, dir, Vector2<>());
      float measuredAngle = -atan2(orthogonalProjectiona.y, orthogonalProjectiona.x);
      measuredAngle = normalize(measuredAngle + (fieldLine.vertical ? pi_2 : 0));
      float possibleAngle2 = normalize(measuredAngle - pi);
      if(abs(normalize(possibleAngle2 - robotPose.rotation)) < abs(normalize(measuredAngle - robotPose.rotation)))
        measuredAngle = possibleAngle2;
      float c = cos(measuredAngle), s = sin(measuredAngle);
      Matrix2x2f angleRotationMatrix(Vector2f(c, s), Vector2f(-s, c));
      Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x, orthogonalProjectiona.y);

      // Check for special object (center circle + center line):
      if(it->midLine && centerCircleCanBeUsed)
      {
        Matrix2x2f circleCov = getCovOfCircle(linePercept.circle.pos, fieldDimensions.centerCircleRadius, motionInfo, cameraMatrix, parameters);
        Matrix2x2f covXR = angleRotationMatrix * cov * angleRotationMatrix.transpose();
        Matrix2x2f covY = angleRotationMatrix * circleCov * angleRotationMatrix.transpose();
        const float measuredX = -orthogonalProjection.x;
        const float xVariance = covXR[0][0];
        const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (it->first - it->last).squareAbs())));
        const float measueredY = -(Pose2D(measuredAngle, measuredX, 0.f) * linePercept.circle.pos).y;

        poseSensorUpdate(Vector3f(measuredX, measueredY, measuredAngle), Matrix3x3f(
          Vector3f(xVariance, 0.f, 0.f), Vector3f(0.f, covY[1][1], 0.f), Vector3f(0.f, 0.f, angleVariance)));
        centerCircleCanBeUsed = false;
      }
      // Integrate "normal" lines:
      else
      {
        cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();
        if(fieldLine.vertical)
        {
          const float measuredY = fieldLine.start.y - orthogonalProjection.y;
          const float yVariance = cov[1][1];
          const float angleVariance = sqr(atan(sqrt(4.f * yVariance / (it->first - it->last).squareAbs())));
          lineSensorUpdate(true, Vector2f(measuredY, measuredAngle), Matrix2x2f(Vector2f(yVariance, 0.f), Vector2f(0.f, angleVariance)));
        }
        else
        {
          const float measuredX = fieldLine.start.x - orthogonalProjection.x;
          const float xVariance = cov[0][0];
          const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (it->first - it->last).squareAbs())));
          lineSensorUpdate(false, Vector2f(measuredX, measuredAngle), Matrix2x2f(Vector2f(xVariance, 0.f), Vector2f(0.f, angleVariance)));
        }
      }
    }
  }
  // There is a valid center circle that has not been fused with the center line:
  if(centerCircleCanBeUsed)
  {
    Matrix2x2f cov = getCovOfCircle(linePercept.circle.pos, fieldDimensions.centerCircleRadius, motionInfo, cameraMatrix, parameters);
    landmarkSensorUpdate(Vector2<>(), Vector2f(linePercept.circle.pos.x, linePercept.circle.pos.y), cov);
  }
  // If there has been a circle, we can stop now:
  if(linePercept.circle.found)
    return;
  // Otherwise, we try to integrate the line intersections:
  for(std::vector<LinePercept::Intersection>::const_iterator intersection = linePercept.intersections.begin();
      intersection != linePercept.intersections.end(); ++intersection)
  {
    const Pose2D robotPose = getPose();
    Vector2<> associatedCorner;
    if(fieldModel.getAssociatedCorner(robotPose, *intersection, associatedCorner))
    {
      Matrix2x2f cov = getCovOfPointInWorld(intersection->pos, 0.f, motionInfo, cameraMatrix, parameters);
      landmarkSensorUpdate(associatedCorner, Vector2f(intersection->pos.x, intersection->pos.y), cov);
    }
  }
}


Matrix2x2f UKFSample::getCovOfPointInWorld(const Vector2<>& pointInWorld2, float pointZInWorld, const MotionInfo& motionInfo,
                                           const CameraMatrix& cameraMatrix, const SelfLocatorParameters& parameters) const
{
  Vector3<> unscaledVectorToPoint = cameraMatrix.invert() * Vector3<>(pointInWorld2.x, pointInWorld2.y, pointZInWorld);
  const Vector3<> unscaledWorld = cameraMatrix.rotation * unscaledVectorToPoint;
  const float h = cameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2f pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2f cossin = distance == 0.f ? Vector2f(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2f rot(cossin, Vector2f(-cossin.y, cossin.x));
  const Vector2<>& robotRotationDeviation = motionInfo.motion == MotionRequest::stand
    ? parameters.robotRotationDeviationInStand : parameters.robotRotationDeviation;
  Matrix2x2f cov(Vector2f(sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - robotRotationDeviation.x) - distance), 0.f),
                  Vector2f(0.f, sqr(tan(robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}


Matrix2x2f UKFSample::getCovOfCircle(const Vector2<>& circlePos, float centerCircleRadius, const MotionInfo& motionInfo,
                                     const CameraMatrix& cameraMatrix, const SelfLocatorParameters& parameters) const
{
  float circleDistance = circlePos.abs();
  Vector2<> increasedCirclePos = circlePos;
  if(circleDistance < centerCircleRadius * 2.f)
  {
    if(circleDistance < 10.f)
      increasedCirclePos = Vector2<>(centerCircleRadius * 2, 0.f);
    else
      increasedCirclePos *= centerCircleRadius * 2.f / circleDistance;
  }
  return getCovOfPointInWorld(increasedCirclePos, 0.f, motionInfo, cameraMatrix, parameters);
}


void UKFSample::landmarkSensorUpdate(const Vector2<>& landmarkPosition, const Vector2f& reading, const Matrix2x2f& readingCov)
{
  generateSigmaPoints();

  // computeLandmarkReadings
  Vector2f landmarkReadings[7];
  for(int i = 0; i < 7; ++i)
  {
    Pose2D pose(sigmaPoints[i].z, sigmaPoints[i].x, sigmaPoints[i].y);
    Vector2<> landmarkPosRel = pose.invert() * landmarkPosition; // TODO: optimize this
    landmarkReadings[i] = Vector2f(landmarkPosRel.x, landmarkPosRel.y);
  }

  // computeMeanOfLandmarkReadings
  Vector2f landmarkReadingMean = landmarkReadings[0];
  for(int i = 1; i < 7; ++i)
    landmarkReadingMean += landmarkReadings[i];
  landmarkReadingMean *= 1.f / 7.f;

  // computeCovOfLandmarkReadingsAndSigmaPoints
  Matrix2x3f landmarkReadingAndMeanCov;
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = landmarkReadings[i * 2 + 1] - landmarkReadingMean;
    landmarkReadingAndMeanCov += Matrix2x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector2f d2 = landmarkReadings[i * 2 + 2] - landmarkReadingMean;
    landmarkReadingAndMeanCov += Matrix2x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  landmarkReadingAndMeanCov *= 0.5f;

  // computeCovOfLandmarkReadingsReadings
  Vector2f d = landmarkReadings[0] - landmarkReadingMean;
  Matrix2x2f landmarkReadingCov(d * d.x, d * d.y);
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = landmarkReadings[i] - landmarkReadingMean;
    landmarkReadingCov += Matrix2x2f(d * d.x, d * d.y);
  }
  landmarkReadingCov *= 0.5f;

  const Matrix3x2f kalmanGain = landmarkReadingAndMeanCov.transpose() * (landmarkReadingCov + readingCov).invert();
  Vector2f innovation = reading - landmarkReadingMean;
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * landmarkReadingAndMeanCov;
}


void UKFSample::lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2x2f& readingCov)
{
  generateSigmaPoints();

  // computeLineReadings
  Vector2f lineReadings[7];
  if(vertical)
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].y, sigmaPoints[i].z);
  else
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].x, sigmaPoints[i].z);

  // computeMeanOfLineReadings
  Vector2f lineReadingMean;
  lineReadingMean = lineReadings[0];
  for(int i = 1; i < 7; ++i)
    lineReadingMean += lineReadings[i];
  lineReadingMean *= 1.f / 7.f;

  // computeCovOfLineReadingsAndSigmaPoints
  Matrix2x3f lineReadingAndMeanCov;
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = lineReadings[i * 2 + 1] - lineReadingMean;
    lineReadingAndMeanCov += Matrix2x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector2f d2 = lineReadings[i * 2 + 2] - lineReadingMean;
    lineReadingAndMeanCov += Matrix2x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  lineReadingAndMeanCov *= 0.5f;

  // computeCovOfLineReadingsReadings
  Matrix2x2f lineReadingCov;
  Vector2f d = lineReadings[0] - lineReadingMean;
  lineReadingCov = Matrix2x2f(d * d.x, d * d.y);
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = lineReadings[i] - lineReadingMean;
    lineReadingCov += Matrix2x2f(d * d.x, d * d.y);
  }
  lineReadingCov *= 0.5f;

  lineReadingMean.y = normalize(lineReadingMean.y);
  const Matrix3x2f kalmanGain = lineReadingAndMeanCov.transpose() * (lineReadingCov + readingCov).invert();
  Vector2f innovation = reading - lineReadingMean;
  innovation.y = normalize(innovation.y);
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * lineReadingAndMeanCov;
}


void UKFSample::poseSensorUpdate(const Vector3f& reading, const Matrix3x3f& readingCov)
{
  generateSigmaPoints();

  // computePoseReadings
  Vector3f poseReadings[7];
  for(int i = 0; i < 7; ++i)
    poseReadings[i] = Vector3f(sigmaPoints[i].x, sigmaPoints[i].y, sigmaPoints[i].z);

  // computeMeanOfPoseReadings
  Vector3f poseReadingMean = poseReadings[0];
  for(int i = 1; i < 7; ++i)
    poseReadingMean += poseReadings[i];
  poseReadingMean *= 1.f / 7.f;

  // computeCovOfPoseReadingsAndSigmaPoints
  Matrix3x3f poseReadingAndMeanCov;
  for(int i = 0; i < 3; ++i)
  {
    Vector3f d1 = poseReadings[i * 2 + 1] - poseReadingMean;
    poseReadingAndMeanCov += Matrix3x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector3f d2 = poseReadings[i * 2 + 2] - poseReadingMean;
    poseReadingAndMeanCov += Matrix3x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  poseReadingAndMeanCov *= 0.5f;

  // computeCovOfPoseReadingsReadings
  Matrix3x3f poseReadingCov;
  Vector3f d = poseReadings[0] - poseReadingMean;
  poseReadingCov = Matrix3x3f(d * d.x, d * d.y, d * d.z);
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = poseReadings[i] - poseReadingMean;
    poseReadingCov += Matrix3x3f(d * d.x, d * d.y, d * d.z);
  }
  poseReadingCov *= 0.5f;

  poseReadingMean.z = normalize(poseReadingMean.z);
  const Matrix3x3f kalmanGain = poseReadingAndMeanCov.transpose() * (poseReadingCov + readingCov).invert();
  Vector3f innovation = reading - poseReadingMean;
  innovation.z = normalize(innovation.z);
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * poseReadingAndMeanCov;
}


void UKFSample::computeWeightingBasedOnValidity(const FieldDimensions& fieldDimensions, const SelfLocatorParameters& parameters)
{
  // Wenn das sample nicht mehr auf dem Teppich ist, ist das weighting gleich NULL
  float factor = computeValidity(fieldDimensions) / validityBuffer.getMaxEntries();
  weighting = parameters.baseValidityWeighting + (1.f - parameters.baseValidityWeighting) * factor;
}


void UKFSample::computeWeightingBasedOnBallObservation(const Vector2<>& ballObservation, const Vector2<>& teamBallPosition,
                                                       const float& camZ, const SelfLocatorParameters& parameters)
{
  // Some constant parameters
  const float distanceObserved = ballObservation.abs();
  const float angleObserved = ballObservation.angle();
  const float distanceAsAngleObserved = (pi_2 - atan2(camZ,distanceObserved));

  // Weighting for original pose
  Pose2D theRobotPose = getPose();
  float originalWeighting = computeAngleWeighting(angleObserved, teamBallPosition, theRobotPose,
    parameters.standardDeviationBallAngle);
  originalWeighting *= computeDistanceWeighting(distanceAsAngleObserved, teamBallPosition, theRobotPose,
    camZ, parameters.standardDeviationBallDistance);

  // Weighting for mirrored pose
  const Pose2D  mirroredPose = Pose2D(pi) + (Pose2D)(theRobotPose);
  float mirroredWeighting = computeAngleWeighting(angleObserved, teamBallPosition, mirroredPose,
    parameters.standardDeviationBallAngle);
  mirroredWeighting *= computeDistanceWeighting(distanceAsAngleObserved, teamBallPosition, mirroredPose,
    camZ, parameters.standardDeviationBallDistance);

  if(originalWeighting >= mirroredWeighting)
    weighting = 1.f;
  else
    weighting = originalWeighting / mirroredWeighting;
}


float UKFSample::computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
  const Pose2D& robotPose, float standardDeviation) const
{
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  return gaussianProbability(abs(modelAngle-measuredAngle), standardDeviation);
}


float UKFSample::computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
  const Pose2D& robotPose, float cameraZ, float standardDeviation) const
{
  const float modelDistance = (robotPose.translation - modelPosition).abs();
  const float modelDistanceAsAngle = (pi_2 - atan2(cameraZ,modelDistance));
  return gaussianProbability(abs(modelDistanceAsAngle-measuredDistanceAsAngle), standardDeviation);
}


Vector2<> UKFSample::getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const
{
  const float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return base + dir * l;
}


void UKFSample::updateMirrorFlag(bool fallen, bool armContact, const SelfLocatorParameters& parameters, const FieldDimensions& fieldDimensions)
{
  const float distToFieldCenter = getPose().translation.abs();
  if(fallen && distToFieldCenter < parameters.maxDistanceToFieldCenterForMirrorActions)
  {
    float probability(0.5f);
    const float rCircle = fieldDimensions.centerCircleRadius;
    if(distToFieldCenter > rCircle)
      probability += ((distToFieldCenter - rCircle) / (parameters.maxDistanceToFieldCenterForMirrorActions - rCircle));
    if(randomFloat() > probability)
    {
      mirrored = !mirrored;
      return;
    }
  }
  if(armContact && distToFieldCenter < parameters.maxDistanceToFieldCenterForMirrorActions)
  {
    float probability(0.7f);
    const float rCircle = fieldDimensions.centerCircleRadius;
    if(distToFieldCenter > rCircle/2)
      probability += ((distToFieldCenter - rCircle) / (parameters.maxDistanceToFieldCenterForMirrorActions - rCircle));
    if(randomFloat() > probability)
    {
      mirrored = !mirrored;
      return;
    }
  }
}


void UKFSample::draw(bool simple)
{
  if(simple)
  {
    Pose2D pose = getPose();
    Vector2<> headPos(30, 0);
    headPos = pose * headPos;
    ColorRGBA sampleFillColor = mirrored ? ColorRGBA(200, 0, 0) : ColorRGBA(200, 200, 0);
    RECTANGLE2("module:SelfLocator:simples", Vector2<>(pose.translation - Vector2<>(55.f, 90.f)), 110, 180, pose.rotation, 0, Drawings::ps_solid,
            ColorRGBA(180, 180, 180),
            Drawings::bs_solid,
            sampleFillColor);
    CIRCLE("module:SelfLocator:simples",
           headPos.x,
           headPos.y,
           30,
           0, // pen width
           Drawings::ps_solid,
           ColorRGBA(180, 180, 180),
           Drawings::bs_solid,
           ColorRGBA(180, 180, 180));
  }
  else
  {
    const float factor = 1.f;
    const float cov012 = cov[0][1] * cov[0][1];
    const float varianceDiff = cov[0][0] - cov[1][1];
    const float varianceDiff2 = varianceDiff * varianceDiff;
    const float varianceSum = cov[0][0] + cov[1][1];
    const float root = sqrt(varianceDiff2 + 4.0f * cov012);
    const float eigenValue1 = 0.5f * (varianceSum + root);
    const float eigenValue2 = 0.5f * (varianceSum - root);
    const float axis1 = 2.0f * sqrt(factor * eigenValue1);
    const float axis2 = 2.0f * sqrt(factor * eigenValue2);
    const float angle = 0.5f * atan2(2.0f * cov[0][1], varianceDiff);
    ELLIPSE("module:SelfLocator:samples", mean, sqrt(3.0f) * axis1, sqrt(3.0f) * axis2, angle,
      10, Drawings::ps_solid, ColorRGBA(100,100,255,100), Drawings::bs_solid, ColorRGBA(100,100,255,100));
    ELLIPSE("module:SelfLocator:samples", mean, sqrt(2.0f) * axis1, sqrt(2.0f) * axis2, angle,
      10, Drawings::ps_solid, ColorRGBA(150,150,100,100), Drawings::bs_solid, ColorRGBA(150,150,100,100));
    ELLIPSE("module:SelfLocator:samples", mean, axis1, axis2, angle,
      10, Drawings::ps_solid, ColorRGBA(255,100,100,100), Drawings::bs_solid, ColorRGBA(255,100,100,100));
  }
}
